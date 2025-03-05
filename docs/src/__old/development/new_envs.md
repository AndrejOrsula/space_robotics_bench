# New Environments

The process of introducing a new environment into the Space Robotics Bench is intended to be modular.

## 1. Duplicate an Existing Environment

Navigate to the [`tasks`](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/space_robotics_bench/tasks) directory, which houses the existing environments. Then, duplicate one of the existing demos or task directories that resembles your desired task/demo more and rename it to the name of your new environment.

## 2. Modify the Environment Configuration

Customize your new environment by altering the configuration files and task implementation code within the folder. This may include asset selection, interaction rules, or specific environmental dynamics.

## 3. Automatic Registration

The new environment will be automatically registered with the Gymnasium API. The environment will be registered under the directory name you assigned during the duplication process.

## 4. Running Your New Environment

Test your new environment by specifying the name of your new environment via the `--env`/`--task`/`--demo` argument.
