import maude
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
import msgType    
import traceback
import argparse
import threading
from time import sleep

 
class RosMaudeNode(Node):
    """Manager for the Maude external object"""

    def __init__(self,manager,name:str):
        super().__init__(name)
        # Queue of events for the caller
        self.oid2publisher = {}
        self.oid2subscription = {}

        self.manager = manager

    def subscription_callback(self,subscription,type):
        def foo(msg):
            msg = msgType.unpack(type,msg)
            while self.oid2subscription[subscription] is not None:
                sleep(0.1)
            self.oid2subscription[subscription] = msg
        return foo

    def run(self, term, data):
        """Receive a message or an update request"""

        try:
            symbol = str(term.symbol())
            reply = None
            if symbol == '%createPublisher':
                dest, sender, datatype, topic, size = term.arguments()

                msgtype = msgType.from_term(datatype)
                topic = topic.prettyPrint(0).strip('"')
                size = size.toInt()

                id = self.manager.freshPublisherOid()
                publisher = self.create_publisher(msgtype,topic,size)
                self.oid2publisher[id] = publisher

                reply = data.getSymbol('createdPublisher')(sender, dest, id)

            elif symbol == '%publish':
                dest, sender, msg = term.arguments()
                publisher = self.oid2publisher[dest]
                msg_type = publisher.msg_type
                msg = msgType.pack(msg_type,msg)
                print('publish',msg)
                publisher.publish(msg)
                reply = data.getSymbol('published')(sender,dest)

            elif symbol == '%createSubscription':
                dest, sender, datatype, topic, size = term.arguments()

                msgtype = msgType.from_term(datatype)
                topic = topic.prettyPrint(0).strip('"')
                size = size.toInt()

                id = self.manager.freshSubscriptionOid()
                self.oid2subscription[id] = None
                callback = self.subscription_callback(id,datatype)
                publisher = self.create_subscription(msgtype,topic,callback,size,callback_group=MutuallyExclusiveCallbackGroup())
                reply = data.getSymbol('createdSubscription')(sender, dest, id)

            elif symbol == '%recieve':
                dest, sender = term.arguments()
                for i in range(10):
                    if (tmp:=self.oid2subscription[dest]) != None:
                        reply = data.getSymbol('recieved')(sender,dest,tmp)
                        self.oid2subscription[dest] = None
                        break
                    sleep(0.1)
                else:
                    reply = term
            else:
                print('Unknown message received:', term, 'with symbol:', symbol)

        except Exception as e:
            traceback.print_exception(e)
        print("reply ",reply)
        return reply

class NodeManager(maude.Hook):
    def __init__(self):
        super().__init__()
        self.inited = False
        self.executors = []
        self.nodes = {}
        self.publisher = None
        self.subscription = None
        self.service = None

        self.publisher_count = 0
        self.subscription_count = 0
        self.service_count = 0
    
    def int2Nat(self,i:int) -> maude.Term:
        return self.NAT.parseTerm(str(i))
        
    def freshPublisherOid(self):
        count = self.publisher.getModule().parseTerm(str(self.publisher_count))
        tmp = self.publisher(count)
        self.publisher_count += 1
        return tmp

    def freshSubscriptionOid(self):
        count = self.subscription.getModule().parseTerm(str(self.subscription_count))
        tmp = self.subscription(count)
        self.subscription_count += 1
        return tmp
    
    def spin(self,node):
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        self.executors.append(executor)
        executor.spin()  # Execute callbacks concurrently
        
    def run(self, term, data):
        """Receive a message or an update request"""
        sleep(0.01)
        print('got:',term)
        try:
            if str(term.symbol()) == '<ROS2-INIT>':
                self.publisher = data.getSymbol('publisher')
                self.subscription = data.getSymbol('subscription')
                self.service = data.getSymbol('service')
                self.inited = True
                return data.getSymbol('inited')()

            if not self.inited : return term
            
            dest, sender, *_ = term.arguments()
            sender.reduce()
            if sender not in self.nodes:
                node = RosMaudeNode(
                    self,
                    "maude_" + sender.prettyPrint(0)
                    )
                self.nodes[sender] = node
                threading.Thread(target=self.spin,args=(node,)).start()
            reply = self.nodes[sender].run(term,data)
        except Exception as e:
            traceback.print_exception(e)
        return reply 

    def done(self):
        for executor in self.executors:
            executor.shutdown()
        for _,node in self.nodes.items():
            node.destroy_node()

def run_logical(file):
    maude.init()
    maude.load(file)

    if (m := maude.getCurrentModule()) is None:
        print('Bad module.')
        exit(1)
    
    header = f"'{str(m)}"
    module_up = maude.getModule('META-LEVEL').parseTerm(f"upModule({header},false)")
    maude.load('ros_logical.maude')
    m = maude.downModule(module_up)

    init = m.parseTerm("init")
    steps = init.rewrite()
    print(f'rew [{steps}] results in:', init)
    
def run_external(file):
    maude.init()
    rclpy.init()
    manager = NodeManager()
    maude.connectRlHook('roshook',manager)
    maude.load(file)

    if (m := maude.getCurrentModule()) is None:
        print('Bad module.')
        exit(1)

    header = f"'{str(m)}"
    module_up = maude.getModule('META-LEVEL').parseTerm(f"upModule({header},false)")
    maude.load('ros_external.maude')
    m = maude.downModule(module_up)
    
    init = m.parseTerm("init")
    result,steps = init.erewrite()
    manager.done()
    print(f'erew [{steps}] results in:', result)
    rclpy.shutdown()

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