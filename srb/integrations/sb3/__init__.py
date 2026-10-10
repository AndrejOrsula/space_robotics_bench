__all__ = ["run"]


def __getattr__(name: str):
    if name == "run":
        from srb.integrations.sb3.main import run

        return run
    raise AttributeError(name)
