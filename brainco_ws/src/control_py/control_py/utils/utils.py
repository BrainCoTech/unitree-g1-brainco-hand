import yaml, importlib, time

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG')

# import Tasks and TasksHandler
def load_mode_tasks(config_path):
    with open(config_path, "r", encoding="utf-8") as f:
        smach_cfg = yaml.safe_load(f)
    mode_cfg = smach_cfg['mode']
    mode = mode_cfg.get("current_mode", "basic")
    
    module_map = mode_cfg.get("mode_modules", {})
    if mode not in module_map:
        raise RuntimeError(f"Unknown mode in config.yaml: {mode}")
    module_path = module_map[mode]

    try:
        # t0 = time.time()
        module = importlib.import_module(module_path)
        # logger.debug(f"import cost: {time.time() - t0}")
    except ImportError as e:
        raise RuntimeError(f"Cannot import module: {module_path}") from e

    Tasks, TasksHandler = None, None

    for attr_name in dir(module):
        attr = getattr(module, attr_name)
        # only match classes
        if not isinstance(attr, type):
            continue
        if attr_name.endswith("Tasks"):
            Tasks = attr
        elif attr_name.endswith("TasksHandler"):
            TasksHandler = attr

    if Tasks is None:
        raise RuntimeError(f"{module_path} must define a *Tasks class")
    if TasksHandler is None:
        raise RuntimeError(f"{module_path} must define a *TasksHandler class")

    # read states config
    states_config_path = mode_cfg["states_config"]["SmFolder"] + mode_cfg["states_config"][mode]

    return mode, Tasks, TasksHandler, states_config_path


def get_action_names(states_config):
    with open(states_config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    
    active_children = {}
    for state in cfg.get('structure', []):
        if isinstance(state, dict) and 'active' in state:
            active_children = state['active'].get('children', {})
            break
        
    return active_children