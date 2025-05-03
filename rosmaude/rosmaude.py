#%%
import maude
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
import traceback
import threading
from ros2interface.api import utilities as ros2util
from time import sleep

primitives = {
    'bool':int,
    'char':str,
    'byte':str,
    'string':str,
    'int8':int,
    'int16':int,
    'int32':int,
    'int64':int,
    'uint8':int,
    'uint16':int,
    'uint32':int,
    'uint64':int,
    'float32':float,
    'float64':float
}

def get_interface(name:str):
    nst=ros2util.get_message_namespaced_type(name)
    return ros2util.import_message_from_namespaced_type(nst)

def make_message_from_dict(interface,d:dict):
    return interface(**dict)

def make_dict_from_message(interface,m):
    ret = interface.get_fields_and_field_types()
    for key in ret:
        ret[key]
    m.__getattribute__(key)
    return 

def decode(s):
    if len(s) >= 2 and s[0] == '"' and s[-1] == '"':
        s = s[1:-1]
    return bytes(s, 'latin-1').decode('unicode_escape')
def encode(s):
    encoded_string = ''.join(f'\\{format(ord(char), "03o")}' for char in s)
    return '"'+encoded_string+'"'

def stringTerm2str(s:maude.Term):
    return decode(s.prettyPrint(0))

def str2stringTerm(m:maude.Module,s:str,wrap=''):
    return m.parseTerm(f"{wrap}({encode(s)})")

def intTerm2int(m:maude.Module,s:maude.Term):
    return s.toInt()

def int2intTerm(m:maude.Module,s:int,wrap=''):
    return m.parseTerm(f"{wrap}({str(s)})")

def floatTerm2float(m:maude.Module,s:maude.Term):
    return s.toFloat()

def float2floatTerm(m:maude.Module,s:float,wrap=""):
    return m.parseTerm(f"{wrap}({str(s)})")

import re
def topicname(s:maude.Term):
    print(s)
    ret = s.prettyPrint(0).strip('"')
    ret = ret.replace("'","")
    ret = ret.replace(".Oid","")
    ret = ret.replace("(","")
    ret = ret.replace(")","")
    ret = ret.replace("[","")
    ret = ret.replace("]","")
    ret = ret.replace(".","_")
    ret = ret.replace(",","_")
    ret = re.sub(r'[^a-zA-Z0-9_/]', '', ret)
    print(ret)
    return ret
    
def iterraw(raw:maude.Term,mapping,cat):
    if raw.symbol()==cat:
        for sub in raw.arguments():
            for key,value in iterraw(sub,mapping,cat):
                yield key,value
        return
    elif raw.symbol()==mapping:
        keyTerm,value = raw.arguments()
        yield stringTerm2str(keyTerm),value

def raw2dict(raw:maude.Term,mapping,cat):
    ret = {}
    rawsort = raw.getsort()
    for key,value in iterraw(raw,mapping,cat):
        if value.getsort() == rawsort:
            value = raw2dict(value,mapping,cat)
        else:
            value = value.arguments()[0]
        ret[key] = value
    return ret

    
def raw2msg(m:maude.Module,interface,raw:maude.Term,mapping,cat):
    # noraw = m.parseTerm('(none).Raw')
    # rawKind = noraw.getSort().kind()
    # cat = m.findSymbol('cat',[rawKind,rawKind],rawKind)
    raw_dict = dict(iterraw(raw,mapping,cat))
    print(raw_dict)
    ret = interface()
    for key, type in ret.get_fields_and_field_types().items():
        assert key in raw_dict
        value = None
        field = raw_dict[key]
        if type in primitives:
            print(field.symbol())
            # assert field.symbol().getName()=='ros#'+type
            data,*_ = field.arguments()
            if primitives[type] == int:
                value = intTerm2int(m,data)
            elif primitives[type] == float:
                value = floatTerm2float(m,data)
            elif primitives[type] == str:
                value = stringTerm2str(data)
        else:
            print('nested msg of type',type)
            interface_sub = get_interface(type)
            value = raw2msg(m,interface_sub,field,mapping,cat)
        setattr(ret,key,value)
    return ret
def msg2raw(m:maude.Module,interface,msg,mapping,cat):
    ret = m.parseTerm('(none).Raw')

    for key, type in interface.get_fields_and_field_types().items():
        value = None 
        if type in primitives:
            op = 'ros#' + type
            if primitives[type] == int:
                value = int2intTerm(m,getattr(msg,key),wrap=op)
            elif primitives[type] == float:
                value = float2floatTerm(m,getattr(msg,key),wrap=op)
            elif primitives[type] == str:
                value = str2stringTerm(m,getattr(msg,key),wrap=op)
        else:
            interface_sub = get_interface(type)
            value = msg2raw(m,interface_sub,getattr(msg,key),mapping,cat)
        ret = cat(ret,mapping(str2stringTerm(m,key),value))
    return ret

class RosMaudeNode(Node):
    """Manager for the Maude external object"""

    def __init__(self,manager,name:str):
        super().__init__(name)
        # Queue of events for the caller
        self.oid2publisher = {}
        self.oid2subscription = {}
        self.oid2subscriptionType = {}

        self.manager = manager

    def subscription_callback(self,subscription):
        def foo(msg):
            self.oid2subscription[subscription].append((msg))
            # data,datatype = self.oid2subscription[subscription] 
            # while data is not None:
            #     sleep(0.1)
            #     data,datatype = self.oid2subscription[subscription] 
            # self.oid2subscription[subscription] = msg,datatype
        return foo

    def run(self, term:maude.Term, data:maude.HookData):
        """Receive a message or an update request"""

        # term.reduce()
        try:
            m = term.symbol().getModule()
            symbol = str(term.symbol())
            reply = None
            if symbol == 'createPublisher':
                dest, sender, datatype, topicT, size = term.arguments()
                msgtype = data.getSymbol("rosType")(datatype)
                msgtype.reduce()
                name = stringTerm2str(msgtype)
                
                interface = get_interface(name)
                topic = topicname(topicT)
                size = size.toInt()

                num = self.manager.freshPublisherNum()
                num = m.parseTerm(str(num))
                id = data.getSymbol('publisher')(num,datatype)

                publisher = self.create_publisher(interface,topic,2)
                self.oid2publisher[id] = publisher,datatype

                reply = data.getSymbol('createdPublisher')(sender, dest, topicT, id)

            elif symbol == 'publish':
                dest, sender, msg = term.arguments()
                publisher,datatype = self.oid2publisher[dest]

                typecheck = data.getSymbol('typecheck')(datatype,msg) 
                typecheck.reduce()
                trueTerm = data.getTerm('true')
                trueTerm.reduce()
                assert typecheck == trueTerm

                # raw = data.getSymbol('upRaw')(msg) 
                # raw.reduce()

                mapping = data.getSymbol('mapping')
                cat = data.getSymbol('cat')
                msg = raw2msg(m,publisher.msg_type,msg,mapping = mapping, cat = cat)
                print('publish',msg)
                print()
                publisher.publish(msg)
                reply = data.getSymbol('published')(sender,dest)

            elif symbol == 'createSubscription':
                dest, sender, datatype, topicT, size = term.arguments()

                msgtype = data.getSymbol("rosType")(datatype)
                msgtype.reduce()
                name = stringTerm2str(msgtype)
                interface = get_interface(name)
                topic = topicname(topicT)
                size = size.toInt()

                num = self.manager.freshSubscriptionNum()
                num = m.parseTerm(str(num))
                id = data.getSymbol('subscription')(num,datatype)
                self.oid2subscription[id] = []
                self.oid2subscriptionType[id] = interface
                callback = self.subscription_callback(id)
                self.create_subscription(interface,topic,callback,10,callback_group=MutuallyExclusiveCallbackGroup())
                reply = data.getSymbol('createdSubscription')(sender, dest, id)
                # self.oid2subscription[id] = None,datatype
                # callback = self.subscription_callback(id)
                # publisher = self.create_subscription(interface,topic,callback,2,callback_group=MutuallyExclusiveCallbackGroup())
                # reply = data.getSymbol('createdSubscription')(sender, dest, id)

            elif symbol == 'recieve':
                dest, sender = term.arguments()
                # _,datatype = self.oid2subscription[dest]
                # msgtype = data.getSymbol("rosType")(datatype)
                # msgtype.reduce()
                # name = stringTerm2str(msgtype)
                # interface = get_interface(name)
                interface = self.oid2subscriptionType[dest]
                mapping = data.getSymbol('mapping')
                cat = data.getSymbol('cat')

                if self.oid2subscription[dest]:
                    msg = self.oid2subscription[dest].pop(0)
                    d = msg2raw(m,interface,msg,mapping=mapping,cat=cat)
                    reply : maude.Term = data.getSymbol('recieved')(sender,dest,d)
                    print('recieve',d.prettyPrint(0))
                    print()
                    print('stack',self.oid2subscription[dest])
                    print()
                else:
                    delay = data.getSymbol('delayrecieve')
                    reply = delay(dest,sender)

                # msg,datatype = self.oid2subscription[dest]
                # if msg != None:
                #     print(msg)
                #     d = msg2raw(m,interface,msg,mapping=mapping,cat=cat)
                #     reply = data.getSymbol('recieved')(sender,dest,d)
                #     self.oid2subscription[dest] = None,datatype
                # else:
                #     delay = data.getSymbol('delayrecieve')
                #     reply = delay(dest,sender)
                    # reply = None
                # for i in range(10):
                #     msg,datatype = self.oid2subscription[dest]
                #     if msg != None:
                #         print(msg)
                #         d = msg2raw(m,interface,msg,mapping=mapping,cat=cat)
                #         # d = data.getSymbol('downRaw')(d)
                #         # d = data.getSymbol("downRaw")(datatype,raw)
                #         reply = data.getSymbol('recieved')(sender,dest,d)
                #         self.oid2subscription[dest] = None,datatype
                #         break
                #     sleep(0.1)
                # else:
                #     reply = term
            else:
                print('Unknown message received:', term, 'with symbol:', symbol)

        except Exception as e:
            traceback.print_exception(e)
        return reply

class NodeManager(maude.Hook):
    def __init__(self):
        super().__init__()
        self.inited = False
        self.executors = []
        self.nodes = {}

        self.publisher_count = 0
        self.subscription_count = 0

    
    def int2Nat(self,i:int) -> maude.Term:
        return self.NAT.parseTerm(str(i))
        
    def freshPublisherNum(self):
        tmp = self.publisher_count
        self.publisher_count += 1
        tmp = maude.getModule('NAT').parseTerm(str(tmp))
        return tmp

    def freshSubscriptionNum(self):
        tmp = self.subscription_count
        self.subscription_count += 1
        tmp = maude.getModule('NAT').parseTerm(str(tmp))
        return tmp
    



    def spin(self,node):
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        self.executors.append(executor)
        executor.spin()  # Execute callbacks concurrently
        
    def run(self, term, data):
        """Receive a message or an update request"""
        # sleep(0.1)
        # print("======================got======================")
        # print('got:',term)
        try:
            dest, sender, *_ = term.arguments()
            sender.reduce()
            if sender not in self.nodes:
                node = RosMaudeNode(
                    self,
                    "maude_" + topicname(sender)
                    )
                self.nodes[sender] = node
                threading.Thread(target=self.spin,args=(node,)).start()
            reply = self.nodes[sender].run(term,data)
        except Exception as e:
            traceback.print_exception(e)
        # print("reply ",reply)
        return reply 

    def done(self):
        for executor in self.executors:
            executor.shutdown()
        for _,node in self.nodes.items():
            node.destroy_node()

def init(m):
    global nodeManager
    rclpy.init()
    manager = NodeManager()
    m.connectRlHook('roshook',manager)
    return manager

def shutdown(manager):
    global nodeManager
    manager.done()
    rclpy.shutdown()