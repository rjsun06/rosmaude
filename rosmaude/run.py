import maude
import rosmaude 
import argparse

def run_logical(file):
    maude.init()
    maude.load(file)

    if (m := maude.getCurrentModule()) is None:
        print('Bad module.')
        exit(1)
    
    maude.load('ros_logical.maude')

    init = m.parseTerm("init <ROS2-Logical>")
    steps = init.rewrite()
    print(f'rew [{steps}] results in:', init)
    
def run_external(file):
    maude.init()
    rm = rosmaude.init(maude)

    maude.load(file)

    if (m := maude.getCurrentModule()) is None:
        print('Bad module.')
        exit(1)
    
    init = m.parseTerm("init")
    result,steps = init.erewrite()

    rosmaude.shutdown(rm)
    print(f'erew [{steps}] results in:', result)

#%%
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='run omod with ros2',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('filename', type=str, 
                       help='Path to the maude file')
    parser.add_argument('-s', '--simulation', action='store_true',
                       help='run simulation without connecting to ros2')
    
    args = parser.parse_args()

    file = args.filename
    if args.simulation:
        run_logical(file)
    else:
        run_external(file)