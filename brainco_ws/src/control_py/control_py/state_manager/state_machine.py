from transitions.extensions import HierarchicalMachine as Machine
from transitions.core import MachineError
from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True)
import yaml

class LifecycleStateMachine:
    def __init__(
        self,
        config_file,
        on_callbacks=None,
        switch_after_done=False,
        trigger_test=True,
        can_switch_active_cb=None,
    ):

        self.trigger_test = trigger_test
        self.switch_after_done = switch_after_done
        self.can_switch_active_cb = can_switch_active_cb

        # 读取 YAML
        with open(config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        self.config = config
        self.callbacks = on_callbacks or {}

        # 从 YAML 生成 active 子状态
        self.substates = []
        self.substate_names = {}  # 存编号 → 名称
        for state in config['structure']:
            if isinstance(state, dict) and 'active' in state:
                children_dict = state['active'].get('children', {})
                self.substates = [str(k) for k in children_dict.keys()]  # 强制转换为字符串
                self.substate_names = {str(k): v for k, v in children_dict.items()}  # 也用字符串做 key
                break

        # 子状态参数初始化
        self.substate_params = {k: {} for k in self.substates}

        # 状态列表
        self.states = []
        for state in config['structure']:
            if isinstance(state, str):
                self.states.append(state)
            elif isinstance(state, dict) and 'active' in state:
                self.states.append({
                    'name': 'active',
                    'initial': str(state['active']['initial']),
                    'children': self.substates
                })

        # 初始化状态机
        self.machine = Machine(model=self, states=self.states, initial=config['initial_state'])
        self.available_transition = {}

        # 添加主状态转换
        main_trans = config.get('main_trans', {})
        allowed_main = config.get('allowed_main_trans', {})
        for src, trans_list in allowed_main.items():
            for trans_id in trans_list:
                trans_name = self._get_trans_name_by_id(main_trans, trans_id)
                if not trans_name:
                    continue
                trans_def = main_trans[trans_name]
                after_cb = self._get_callback(trans_def.get('after'))
                before_cb = self._get_callback(trans_def.get('before'))
                src_val = trans_def['src']
                if src_val == "any":
                    src_val = "*"  # 保留字符串支持
                self.add_trans(trans_name, src_val, trans_def['dst'], after=after_cb, before=before_cb)

        # 添加子状态转换（遵循 allowed_sub_trans）
        allowed_sub = config.get('allowed_sub_trans', {})
        for src in self.substates:
            dst_list = self._resolve_allowed_sub_targets(src, allowed_sub.get(src, "all"))
            for dst in dst_list:
                self.add_trans(
                    f'start_{dst}',
                    f'active_{src}',
                    f'active_{dst}',
                    after=self._get_callback(f'on_active_{dst}')
                )

        # 动态生成 on_active_X 方法
        for i in self.substates:
            setattr(self.__class__, f'on_active_{i}', self._make_active_handler(i))

    # ---------- 辅助方法 ----------
    def _get_callback(self, name):
        return self.callbacks.get(name)

    def _get_trans_name_by_id(self, main_trans, id_):
        for name, trans_def in main_trans.items():
            if trans_def.get('id') == id_:
                return name
        return None

    def _make_active_handler(self, index):
        def handler(self):
            cb = self.callbacks.get(f'on_active_{index}')
            if cb:
                cb()
        return handler

    def _resolve_allowed_sub_targets(self, src, dst_spec):
        src = str(src)

        if dst_spec is None:
            dst_spec = "all"

        if isinstance(dst_spec, str):
            normalized = dst_spec.strip().lower()
            if normalized == "all":
                return [dst for dst in self.substates if dst != src]
            if normalized in ("all_self", "all-with-self", "all_with_self"):
                return list(self.substates)
            return [normalized]

        return [str(dst) for dst in dst_spec]

    def add_trans(self, trans, src, dst, after=None, before=None):
        self.machine.add_transition(trans, src, dst, after=after, before=before)
        if src not in self.available_transition:
            self.available_transition[src] = []
        if (trans, dst) not in self.available_transition[src]:
            self.available_transition[src].append((trans, dst))

    def _is_active_sub_transition(self, event_name):
        return isinstance(event_name, str) and event_name.startswith('start_')

    def _is_active_state(self, state_name):
        return (
            isinstance(state_name, str)
            and (state_name.startswith('active.') or state_name.startswith('active_'))
        )

    def _get_active_substate(self, state_name):
        if not self._is_active_state(state_name):
            return None
        if '.' in state_name:
            return state_name.split('.', 1)[1]
        if '_' in state_name:
            return state_name.split('_', 1)[1]
        return None

    def _can_run_event(self, event_name):
        if not self._is_active_sub_transition(event_name):
            return True, ""
        if not self._is_active_state(self.state):
            return True, ""

        current_substate = self._get_active_substate(self.state)
        if self.can_switch_active_cb is None:
            return True, ""
        cb_result = self.can_switch_active_cb(self.state, event_name)
        if isinstance(cb_result, tuple):
            allowed, reason = cb_result
        else:
            allowed, reason = bool(cb_result), ""

        if allowed:
            return True, ""

        return False, (
            reason
            or (
                f"Current action '{current_substate}' is still running. "
                f"Wait until it is done before triggering '{event_name}'."
            )
        )

    def get_state(self):
        return self.state

    def get_substate_param(self):
        substate = self._get_active_substate(self.state)
        if substate is not None:
            return self.substate_params.get(substate)
        return None

    def trigger_event(self, event_name, param=None):
        if not hasattr(self, event_name):
            return False, f"Event '{event_name}' does not exist."

        allowed, block_message = self._can_run_event(event_name)
        if not allowed:
            if self._is_active_sub_transition(event_name):
                logger.warning(block_message)
            return False, block_message

        trigger = getattr(self, event_name)

        if self.trigger_test:
            trigger()
        else:
            try:
                trigger()
            except MachineError as e:
                logger.warning(f"{e}\nPlease check the states config file.")
                return False, f"Trigger event failed"
            except Exception as e:
                return False, f"Error: {str(e)}"

        substate = self._get_active_substate(self.state)
        if substate is not None and param is not None:
            self.substate_params[substate] = param

        return True, f"Triggered event '{event_name}' → New state: {self.state}"

    def get_available_events(self):
        available_list = []
        try:
            available = self.available_transition[self.state]
        except KeyError:
            return []
        for trans, dst in available:
            allowed, _ = self._can_run_event(trans)
            if not allowed:
                continue
            available_list.append(f"{trans}->{dst}")
        return sorted(available_list)

    def debug_print_all_transitions(self):
        print("\n[DEBUG] 全部事件与状态转换源:")
        for event_name, event in self.machine.events.items():
            print(f"  Event: {event_name}")
            for src, transitions in event.transitions.items():
                dests = [t.dest for t in transitions]
                print(f"    from: {src} → {dests}")
