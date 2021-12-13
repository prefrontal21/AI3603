from .sokoban_env import SokobanEnv

class SokobanEnv_Small0(SokobanEnv):
    metadata = {
        'render.modes': ['human', 'rgb_array', 'tiny_human', 'tiny_rgb_array'],
    }

    def __init__(self, **kwargs):
        kwargs['dim_room'] = kwargs.get('dim_room', (7, 7))
        kwargs['max_steps'] = kwargs.get('max_steps', 200)
        kwargs['num_boxes'] = kwargs.get('num_boxes', 2)
        super(SokobanEnv_Small0, self).__init__(**kwargs)


class SokobanEnv_Small1(SokobanEnv):
    metadata = {
        'render.modes': ['human', 'rgb_array', 'tiny_human', 'tiny_rgb_array'],
    }

    def __init__(self, **kwargs):
        kwargs['dim_room'] = kwargs.get('dim_room', (7, 7))
        kwargs['max_steps'] = kwargs.get('max_steps', 200)
        kwargs['num_boxes'] = kwargs.get('num_boxes', 3)
        super(SokobanEnv_Small1, self).__init__(**kwargs)

class SokobanEnv_Small2(SokobanEnv):
    """FOR HW2 AI3603 CLASS"""
    metadata = {
        'render.modes': ['human', 'rgb_array', 'tiny_human', 'tiny_rgb_array'],
    }

    def __init__(self, **kwargs):
        kwargs['dim_room'] = kwargs.get('dim_room', (7, 7))
        kwargs['max_steps'] = kwargs.get('max_steps', 100) # 100
        kwargs['num_boxes'] = kwargs.get('num_boxes', 2)
        super(SokobanEnv_Small2, self).__init__(**kwargs)
