import queue
import threading
from types import TracebackType

import embodied
import numpy as np


class RealTimeDriver(embodied.Driver):
    def __init__(self, env, agent):
        self.env = env
        self.agent = agent

        self.callbacks = []
        self.acts = None
        self.carry = None

        # Async State
        self._queue = queue.Queue(maxsize=1000)
        self._lock = threading.Lock()
        self._latest_vars = None  # Stores the state dict
        self._failure_lock = threading.Lock()
        self._count_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._stop_event = threading.Event()
        # Set once the actor can enqueue nothing more (it exited, or close()
        # gave up on it); the learner drains the queue until then.
        self._actor_done = threading.Event()
        self._learner_failure_event = threading.Event()
        self._learner_failure: tuple[BaseException, TracebackType | None] | None = None
        self._cleanup_ledger = None

        self._running = False
        self._threads_started = False

        # Stats
        self._step_count = 0
        self._episode_count = 0
        self._init_state = None

        # Init placeholders
        self._init_act = None

    def _stop(self):
        self._running = False
        self._stop_event.set()

    def _record_learner_failure(self, error: BaseException):
        with self._failure_lock:
            if self._learner_failure is not None:
                return
            self._learner_failure = (error, error.__traceback__)
            self._stop()
            self._learner_failure_event.set()

    def record_external_failure(self, error: BaseException) -> None:
        """Make an owned companion worker failure session-fatal."""
        self._record_learner_failure(error)

    def _get_learner_failure(self):
        with self._failure_lock:
            return self._learner_failure

    @staticmethod
    def _raise_failure(failure):
        error, traceback = failure
        raise error.with_traceback(traceback)

    def _raise_learner_failure(self):
        failure = self._get_learner_failure()
        if failure is not None:
            self._raise_failure(failure)

    def _check_thread_liveness(self):
        if not self._learner_thread.is_alive():
            self._record_learner_failure(
                RuntimeError("Learner thread died unexpectedly.")
            )
            self._raise_learner_failure()

        if not self._actor_thread.is_alive():
            self._raise_learner_failure()
            self._stop()
            raise RuntimeError("Actor thread died unexpectedly.")

    def reset(self, init_policy=None):
        if init_policy:
            self.carry = init_policy(1)
            self._init_state = self.carry

    def on_step(self, callback):
        self.callbacks.append(callback)

    def close(self):
        from srb.integrations.dreamer.cleanup import (
            CleanupLedger,
            annotate_primary,
            join_thread_bounded,
        )

        with self._close_lock:
            ledger = getattr(self, "_cleanup_ledger", None)
            if ledger is None:
                ledger = self._cleanup_ledger = CleanupLedger()

        cleanup_errors: list[BaseException] = []

        def _join(thread_name: str, timeout_s: float) -> None:
            error = join_thread_bounded(
                thread_name, getattr(self, thread_name, None), timeout_s=timeout_s
            )
            if error is not None:
                raise error

        def _stop_actor() -> None:
            # Stop the producer first. The learner then drains every transition
            # already accepted into the bounded queue before it exits.
            try:
                _join("_actor_thread", 2.0)
                return
            except BaseException as error:
                actor_error = error
            # The actor is still inside a hardware step. Latch the environment's
            # safe stop so that step (and any later one) fails closed, then give
            # the thread one more bounded chance to unwind before env.close()
            # tears the transports down underneath it.
            abort = getattr(self.env, "abort_session", None)
            if callable(abort):
                abort("dreamer driver closing while a hardware step is in flight")
            try:
                _join("_actor_thread", 2.0)
            except BaseException as retry_error:
                raise retry_error from actor_error
            raise actor_error

        def _stop_actor_then_release_learner() -> None:
            try:
                _stop_actor()
            finally:
                self._actor_done.set()

        self._stop()
        errors = ledger.run_once(
            [
                ("actor_thread", _stop_actor_then_release_learner),
                ("learner_thread", lambda: _join("_learner_thread", 2.0)),
                ("env.close", self.env.close),
            ]
        )
        cleanup_errors.extend(error for _name, error in errors)

        failure = self._get_learner_failure()
        if failure is not None:
            annotate_primary(failure[0], ledger.errors)
            self._raise_failure(failure)
        if cleanup_errors:
            raise cleanup_errors[0]

    def update_variables(self, new_vars):
        """Called by learner thread to update weights in-memory."""
        with self._lock:
            self._latest_vars = new_vars

    def __call__(self, policy, steps=0, episodes=0):
        if not self._threads_started:
            self._start_threads(policy)

        with self._count_lock:
            target_steps = self._step_count + steps
            target_episodes = self._episode_count + episodes

        self._raise_learner_failure()
        while True:
            self._raise_learner_failure()
            self._check_thread_liveness()
            with self._count_lock:
                target_reached = not (
                    self._step_count < target_steps
                    or self._episode_count < target_episodes
                )
            if target_reached:
                break
            self._learner_failure_event.wait(timeout=0.05)

        self._raise_learner_failure()
        self._check_thread_liveness()

    def _start_threads(self, policy):
        print("[RealTimeDriver] Warming up JAX policy...")
        self._warmup(policy)
        print("[RealTimeDriver] Starting Async Threads.")

        self._stop_event.clear()
        self._actor_done.clear()
        self._learner_failure_event.clear()
        self._running = True

        self._learner_thread = threading.Thread(
            target=self._learner_loop, daemon=False, name="LearnerThread"
        )
        self._learner_thread.start()

        self._actor_thread = threading.Thread(
            target=self._actor_loop, args=(policy,), daemon=False, name="ActorThread"
        )
        self._actor_thread.start()

        self._threads_started = True

    def _warmup(self, policy):
        obs = self.env.reset()
        if self._init_state is None:
            self.reset(lambda n: self.agent.init_policy(n))
        obs_clean = {k: v for k, v in obs.items() if not k.startswith("log/")}
        _, action, _outs = policy(self._init_state, obs_clean)

        self._init_act = {k: np.zeros_like(v) for k, v in action.items()}
        if isinstance(self._init_act, dict):
            self._init_act["reset"] = np.ones(1, dtype=bool)

    def _actor_loop(self, policy):
        try:
            self._actor_loop_impl(policy)
        except BaseException as error:
            if self._get_learner_failure() is None:
                self._record_learner_failure(error)
        finally:
            self._actor_done.set()

    def _actor_loop_impl(self, policy):
        action = self._init_act
        agent_state = self._init_state

        while self._running and not self._stop_event.is_set():
            # 1. Sync Weights (In-Memory)
            vars_to_load = None
            with self._lock:
                if self._latest_vars is not None:
                    vars_to_load = self._latest_vars
                    self._latest_vars = None

            if vars_to_load is not None:
                # agent.load() updates the ACTOR agent safely
                self.agent.load(vars_to_load)

            if self._stop_event.is_set():
                break

            # 2. Action Processing
            phys_action = (
                {k: v[0] for k, v in action.items()}
                if isinstance(action, dict)
                else action[0]
            )
            if isinstance(phys_action, dict):
                phys_action = dict(phys_action)
                if "reset" not in phys_action:
                    phys_action["reset"] = np.array(False)

            if self._stop_event.is_set():
                break

            # 3. Hardware Step
            obs = self.env.step(phys_action)

            # No stop check from here until the transition is enqueued: once
            # the policy has chosen the action for this observation, the step's
            # data is enqueued even if a stop arrived meanwhile. (If the policy
            # itself fails, the observation is lost with the session.)
            logs = {k: v for k, v in obs.items() if k.startswith("log/")}
            obs_clean = {k: v for k, v in obs.items() if not k.startswith("log/")}

            # 4. The is_last observation still belongs to its episode, so keep
            # the carry for its replay entries (enc/dyn/dec with replay_context).
            # Dreamer resets the policy on the next is_first observation, as in
            # embodied.Driver._step.
            is_last = np.atleast_1d(
                np.logical_or(
                    obs.get("is_last", False),
                    obs.get("is_terminal", False),
                )
            )

            # 5. Inference
            agent_state, action, outs = policy(agent_state, obs_clean)

            # 6. Record obs_t with its chosen action, matching embodied.Driver:
            # zero it on is_last and set reset=is_last for the next command.
            if isinstance(action, dict):
                action = {k: v for k, v in action.items() if k != "reset"}
                if np.any(is_last):
                    action = {k: self._mask(v, ~is_last) for k, v in action.items()}
            tran = {**obs_clean, **action, **outs, **logs}
            self._enqueue_transition(tran)

            if isinstance(action, dict):
                action = dict(action)
                action["reset"] = np.array(is_last, dtype=bool)

    @staticmethod
    def _mask(value, mask):
        value = np.asarray(value)
        while mask.ndim < value.ndim:
            mask = mask[..., None]
        return value * mask.astype(value.dtype)

    def _enqueue_transition(self, tran) -> None:
        """Apply bounded backpressure; never silently discard hardware data."""
        while True:
            self._raise_learner_failure()
            try:
                self._queue.put(tran, timeout=0.05)
                return
            except queue.Full:
                pass
            # Only reject once the queue is really full. A stop request must
            # not discard the transition of a hardware step already in flight;
            # the learner drains the queue after the stop event is set.
            stop_event = getattr(self, "_stop_event", None)
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError(
                    "transition queue is full and the driver is stopping; the "
                    "transition was not accepted"
                )
            learner = getattr(self, "_learner_thread", None)
            if learner is not None and not learner.is_alive():
                raise RuntimeError(
                    "transition queue is full after learner thread stopped"
                )

    def _learner_loop(self):
        try:
            while True:
                try:
                    tran_batch = self._queue.get(timeout=0.1)
                except queue.Empty:
                    # Exit only once the actor can add nothing more: a stop can
                    # land while it is still inside env.step or the policy, and
                    # its last transition must still be drained. The actor sets
                    # _actor_done after its final put, so an empty queue seen
                    # after that flag is final.
                    if (
                        self._stop_event.is_set()
                        and self._actor_done.is_set()
                        and self._queue.empty()
                    ):
                        break
                    continue

                tran = {k: v[0] for k, v in tran_batch.items()}
                tran.pop("reset", None)

                for fn in self.callbacks:
                    fn(tran, 0)

                with self._count_lock:
                    self._step_count += 1
                    if tran["is_last"]:
                        self._episode_count += 1
        except BaseException as error:
            self._record_learner_failure(error)
