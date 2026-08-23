from threading import Thread

from ros_actor import register_subsystem, run_actor, init_server, init_spin, shutdown_server
from ros_actor.command import CommandInterpreter

from lib.actor.system import Tb3, MapSystem

def cm_init(node):
    register_subsystem('robot', Tb3)
    init_spin(node)
    run_actor('init_realsense')
    # register_subsystem('map', MapSystem)
    run_actor('make_symbolic_link')
    run_actor('update_bt')
    Thread(target=CommandInterpreter().do_command).start()

def main():
    init_server(cm_init)
    shutdown_server()

if __name__ == '__main__':
    main()
