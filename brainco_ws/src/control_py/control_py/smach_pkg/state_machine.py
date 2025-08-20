from transitions.extensions import HierarchicalMachine as Machine

class LifecycleStateMachine:
    def __init__(self, action_num,
                 on_configure_callback=None,
                 on_shutdown_callback=None,
                 **kwargs):

        # 回调统一管理
        self.callbacks = {
            'configure': on_configure_callback,
            'shutdown': on_shutdown_callback
        }

        # 绑定 active 回调
        self.num_active_states = action_num 
        for i in range(self.num_active_states):
            self.callbacks[f'active_{i}'] = kwargs.get(f'on_active_{i}_callback', None)

        # 子状态参数
        self.substate_params = {str(i): f'param{i}' for i in range(self.num_active_states)}
        self.substates = list(self.substate_params.keys())
        self.substate_trans_str = 'start_'

        # 状态定义
        self.states = [
            'unconfigured',
            'inactive',
            {
                'name': 'active',
                'initial': '0',
                'children': self.substates
            },
            'finalized'
        ]

        # 初始化状态机
        self.machine = Machine(model=self, states=self.states, initial='unconfigured')

        # 只显式列出允许的状态
        self.available_transition = {}

        # 主状态转换
        self.add_trans('configure', 'unconfigured', 'inactive', after='on_configure')
        self.add_trans('activate', 'inactive', 'active', after='on_active_0')
        self.add_trans('deactivate', 'active', 'inactive', after='on_configure')
        self.add_trans('cleanup', 'inactive', 'unconfigured')
        self.add_trans('shutdown', '*', 'finalized', before='on_shutdown')

        # 子状态切换转换
        for src in self.substates:
            for dst in self.substates:
                if src != dst:
                    self.add_trans(
                        self.substate_trans_str + dst,
                        f'active_{src}',
                        f'active_{dst}',
                        after=f'on_active_{dst}'
                    )

        # 动态生成 on_active_X 方法
        for i in range(self.num_active_states):
            setattr(self.__class__, f'on_active_{i}', self._make_active_handler(i))

    def _make_active_handler(self, index):
        # 返回一个绑定到类的方法
        def handler(self):
            cb = self.callbacks.get(f'active_{index}', None)
            if cb:
                cb()
        return handler

    def on_configure(self):
        cb = self.callbacks.get('configure', None)
        if cb:
            cb()

    def on_shutdown(self):
        cb = self.callbacks.get('shutdown', None)
        if cb:
            cb()
    
        
    def add_trans(self, trans, src, dst, after=None, before=None):
        self.machine.add_transition(trans, src, dst, after=after, before=before)
        if src in self.available_transition:
            self.available_transition[src].append((trans, dst))
        else:
            self.available_transition[src] = [(trans, dst)]
        if "active_" in src and ('deactivate', 'inactive') not in self.available_transition[src]:
            self.available_transition[src].append(('deactivate', 'inactive'))
        if ('shutdown', 'finalized') not in self.available_transition[src]:
            self.available_transition[src].append(('shutdown', 'finalized'))
    
    def get_state(self):
        """返回当前状态名称"""
        return self.state

    def get_substate_param(self):
        """返回当前子状态绑定的参数（仅在 active.substateX 状态有效）"""
        if self.state.startswith('active.'):
            substate = self.state.split('.')[1]
            return self.substate_params.get(substate)
        return None

    def trigger_event(self, event_name, param=None):
        """
        通用触发事件接口，可传入用于状态切换的参数
        :param data: 触发的事件名（如 'configure'）
        :param param: 可选的字符串参数，用于更新子状态参数
        """
        if not hasattr(self, event_name):
            return False, f"Event '{event_name}' does not exist."

        trigger = getattr(self, event_name)

        try:
            result = trigger()
        except Exception as e:
            return False, f"Error: {str(e)}"

        # 如果是切换到 active 子状态，更新参数
        if self.state.startswith("active.") and param is not None:
            substate = self.state.split('.')[1]
            self.substate_params[substate] = param

        return True, f"triggered event '{event_name}' → new state: {self.state}"

    def get_available_events(self):
        available_list = []
        available = self.available_transition[self.state]
        
        for trans, dst in available:
            trans_str = trans + "->" + dst
            available_list.append(trans_str)
        return sorted(available_list)

    def debug_print_all_transitions(self):
        print("\n[DEBUG] 全部事件与状态转换源:")
        for event_name, event in self.machine.events.items():
            print(f"  Event: {event_name}")
            for src, transitions in event.transitions.items():
                dests = [t.dest for t in transitions]
                print(f"    from: {src} → {dests}")
