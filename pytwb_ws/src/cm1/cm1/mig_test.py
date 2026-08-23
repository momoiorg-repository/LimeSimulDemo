# Auto-generated from @actor functions
# Do not edit manually

from some_module import ActorBT, behavior

@behavior
class Carib(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'carib')


@behavior
class Object_loc(ActorBT):
    def __init__(self, name, node, target='link1'):
        super().__init__(name, 'object_loc', target='link1')


@behavior
class Object_front(ActorBT):
    def __init__(self, name, node, target='link1'):
        super().__init__(name, 'object_front', target='link1')


@behavior
class Object_glance(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'object_glance')


@behavior
class Get_found(ActorBT):
    def __init__(self, name, node, max_time=10, min_count=10):
        super().__init__(name, 'get_found', max_time=10, min_count=10)


@behavior
class Measure_center(ActorBT):
    def __init__(self, name, node, target='link1', assumed=0.28, log=None):
        super().__init__(name, 'measure_center', target='link1', assumed=0.28, log=None)


@behavior
class Measure_center2(ActorBT):
    def __init__(self, name, node, assumed=0.25, log=None):
        super().__init__(name, 'measure_center2', assumed=0.25, log=None)


@behavior
class Center_angle(ActorBT):
    def __init__(self, name, node, assumed=0.25):
        super().__init__(name, 'center_angle', assumed=0.25)


@behavior
class Find_object(ActorBT):
    def __init__(self, name, node, minus):
        super().__init__(name, 'find_object', minus)

@behavior
class Pic_find(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'pic_find')


@behavior
class Coke_getter(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'coke_getter')


@behavior
class Cdisp(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'cdisp')


@behavior
class Read_marker(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'read_marker')


@behavior
class Set_detector(ActorBT):
    def __init__(self, name, node, full_name, n=None):
        super().__init__(name, 'set_detector', full_name, n=None)


@behavior
class Set_func(ActorBT):
    def __init__(self, name, node, full_name):
        super().__init__(name, 'set_func', full_name)


@behavior
class Use_func(ActorBT):
    def __init__(self, name, node):
        super().__init__(name, 'use_func')

