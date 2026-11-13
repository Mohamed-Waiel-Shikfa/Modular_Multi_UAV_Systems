"""Task registry: cfg.task -> run(cfg) -> exit code (0 = every check passed)."""


def run(cfg):
    if cfg.task == "dock":
        from modular_uav.dock.world import run as task
    else:
        from modular_uav.tasks.hover import run as task
    return task(cfg)
