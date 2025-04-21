import maude
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
import ctypes
import traceback
import argparse
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

def iterraw(raw:maude.Term,cat=None):
    m = raw.symbol().getModule()
    if raw.symbol()!=cat:
        key,value = raw.arguments()
        yield stringTerm2str(key),value
        return
    for field in raw.arguments():
        print(field)
        key,value = field.arguments()
        yield stringTerm2str(key),value

def raw2dict(raw:maude.Term,cat=None):
    ret = {}
    rawsort = raw.getsort()
    for key,value in iterraw(raw,cat):
        if value.getsort() == rawsort:
            value = raw2dict(value)
        else:
            value,*_ = value.arguments()
        ret[key] = value
    return ret

    
def raw2msg(m:maude.Module,interface,raw:maude.Term):
    noraw = m.parseTerm('(none).Raw')
    rawKind = noraw.getSort().kind()
    cat = m.findSymbol('cat',[rawKind,rawKind],rawKind)
    raw_dict = dict(iterraw(raw,cat=cat))
    ret = interface()
    for key, type in ret.get_fields_and_field_types().items():
        assert key in raw_dict
        value = None
        if type in primitives:
            field = raw_dict[key]
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
            interface_sub = get_interface(type)
            value = raw2msg(m,interface_sub,raw_dict[field])
        setattr(ret,key,value)
    return ret
#%%
def msg2raw(m:maude.Module,interface,msg):
    ret = m.parseTerm('(none).Raw')
    rawKind = ret.getSort().kind()
    cat = m.findSymbol('cat',[rawKind,rawKind],rawKind)
    stringKind = m.parseTerm('""').getSort().kind()
    mapping = m.findSymbol('mapping',[stringKind,rawKind],rawKind)

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
            value = msg2raw(m,interface_sub,getattr(msg,key))
        ret = cat(ret,mapping(str2stringTerm(m,key),value))
    return ret

def castObj(term:maude.Term):
    term.reduce()
    id,*_ = term.arguments()
    id = id.toInt()
    ret = ctypes.cast(id,ctypes.py_object).value
    return ret

class RosMaudeNode(Node):
    """Manager for the Maude external object"""

    def __init__(self,manager,name:str):
        super().__init__(name)
        # Queue of events for the caller
        self.oid2publisher = {}
        self.oid2subscription = {}

        self.manager = manager

    def subscription_callback(self,subscription):
        def foo(msg):
            data,datatype = self.oid2subscription[subscription] 
            while data is not None:
                sleep(0.1)
                data,datatype = self.oid2subscription[subscription] 
            self.oid2subscription[subscription] = msg,datatype
        return foo

    def run(self, term:maude.Term, data:maude.HookData):
        """Receive a message or an update request"""

        try:
            m = term.symbol().getModule()
            symbol = str(term.symbol())
            reply = None
            if symbol == 'createPublisher':
                dest, sender, datatype, topic, size = term.arguments()
                msgtype = data.getSymbol("rosType")(datatype)
                msgtype.reduce()
                # print(msgtype)
                interface = self.manager.get(msgtype)
                # print(interface)
                topic = topic.prettyPrint(0).strip('"')
                size = size.toInt()

                num = self.manager.freshPublisherNum()
                num = m.parseTerm(str(num))
                id = data.getSymbol('publisher')(num,datatype)

                publisher = self.create_publisher(interface,topic,size)
                self.oid2publisher[id] = publisher,datatype

                reply = data.getSymbol('createdPublisher')(sender, dest, id)

            elif symbol == 'publish':
                dest, sender, msg = term.arguments()
                publisher,datatype = self.oid2publisher[dest]

                msg = self.manager.get(msg)
                print('publishing',msg)
                publisher.publish(msg)
                reply = data.getSymbol('published')(sender,dest)

            elif symbol == 'createSubscription':
                dest, sender, datatype, topic, size = term.arguments()

                msgtype = data.getSymbol("rosType")(datatype)
                msgtype.reduce()
                interface = self.manager.get(msgtype)
                # print('interface',interface)
                topic = topic.prettyPrint(0).strip('"')
                size = size.toInt()

                num = self.manager.freshSubscriptionNum()
                num = m.parseTerm(str(num))
                id = data.getSymbol('subscription')(num,datatype)
                self.oid2subscription[id] = None,datatype
                callback = self.subscription_callback(id)
                publisher = self.create_subscription(interface,topic,callback,size,callback_group=MutuallyExclusiveCallbackGroup())
                reply = data.getSymbol('createdSubscription')(sender, dest, id)

            elif symbol == 'recieve':
                dest, sender = term.arguments()
                # _,datatype = self.oid2subscription[dest]
                # msgtype = data.getSymbol("rosType")(datatype)
                # msgtype.reduce()
                # name = stringTerm2str(msgtype)
                # interface = get_interface(name)
                for i in range(10):
                    msg,datatype = self.oid2subscription[dest]
                    if msg != None:
                        print(msg)
                        # raw = msg2raw(m,interface,msg)
                        # d = data.getSymbol("downRaw")(datatype,raw)
                        self.oid2subscription[dest] = None,datatype
                        msgId = self.manager.reg(msg)
                        intTerm = m.parseTerm(str(msgId))
                        d = data.getSymbol("ptr")(intTerm)
                        reply = data.getSymbol('recieved')(sender,dest,d)
                        break
                    sleep(0.1)
                else:
                    reply = term
            else:
                print('Unknown message received:', term, 'with symbol:', symbol)

        except Exception as e:
            traceback.print_exception(e)
        return reply

class NodeManager(maude.Hook):
    def __init__(self,reg,get):
        super().__init__()
        self.inited = False
        self.executors = []
        self.nodes = {}

        self.publisher_count = 0
        self.subscription_count = 0
        self.reg = reg
        self.get = get

    
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
        sleep(0.01)
        print('got:',term)
        try:
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
        print("reply ",reply)
        return reply 

    def done(self):
        for executor in self.executors:
            executor.shutdown()
        for _,node in self.nodes.items():
            node.destroy_node()


class MessageManager(maude.Hook):
    def __init__(self,reg,get):
        super().__init__()
        self.reg = reg
        self.get = get
        
    def run(self, term:maude.Term, data:maude.HookData):
        print("got", term)
        obj = self._run(term, data)
        ptr = data.getSymbol("ptr")
        m = term.symbol().getModule()
        # print('reg')
        id = self.reg(obj)
        # print(id)
        intTerm = m.parseTerm(str(id))
        # print("got", term, 'rep', obj)
        return ptr(intTerm)

    def _run(self, term:maude.Term, data:maude.HookData):
        symbol = str(term.symbol())
        args = term.arguments()
        reply = None
        # print("got symbol",symbol)
        if symbol == "msgType":
            interface_name,*_ = args
            # print('interface name term',interface_name)
            interface_name = stringTerm2str(interface_name)
            # print('interface name',interface_name)
            reply = get_interface(interface_name)
        elif keyTerm:=data.getTerm("key"):
            # print('key access',keyTerm)
            msg,_ = args
            msg = self.get(msg)
            key = stringTerm2str(keyTerm)
            reply = getattr(msg,key)
        elif keysTerm:=data.getTerm("keys"):
            rosType=data.getTerm("rosType")
            rosType.reduce()
            print('msg init',rosType)
            interface = self.get(rosType)
            print('msg init',interface)
            # print(stringTerm2str(keysTerm).split(' '))
            # arg,*_ = args
            # print(arg)
            # arg.reduce()
            # print(castObj(arg))
            # print(*map(castObj,args))
            args = dict(zip(stringTerm2str(keysTerm).split(' '),map(self.get,args)))
            print(args)
            reply = interface(**args)
        print('rep',reply)
        return reply

class ValueManager(maude.Hook):
    def __init__(self,reg,get):
        super().__init__()
        self.reg = reg
        self.get = get

    def run(self, term:maude.Term, data:maude.HookData):
        symbol = str(term.symbol())
        m = term.symbol().getModule()
        value,*_ = term.arguments()
        reply = None
        if symbol == 'int':
            value = self.get(value)
            reply = m.parseTerm(str(value))
        if symbol == 'float':
            value = self.get(value)
            reply = m.parseTerm(str(value))
        if symbol == 'string':
            value = self.get(value)
            reply =  m.parseTerm(encode(value))
        obj = self._run(term, data)
        ptr = data.getSymbol("ptr")
        m = term.symbol().getModule()
        intTerm = m.parseTerm(str(self.reg(obj)))
        reply = ptr(intTerm)
        return reply

    def _run(self, term:maude.Term, data):
        symbol = str(term.symbol())
        value, *_ = term.arguments()
        if 'ros#int' in symbol:
            return value.toInt()
        if 'ros#float' in symbol:
            return value.tofloat()
        if 'ros#string' in symbol:
            return stringTerm2str(value)
        if 'ros#byte' in symbol:
            return stringTerm2str(value)


def run_logical(file):
    maude.init()
    maude.load(file)

    if (m := maude.getCurrentModule()) is None:
        print('Bad module.')
        exit(1)
    
    # header = f"'{str(m)}"
    # module_up = maude.getModule('META-LEVEL').parseTerm(f"upModule({header},false)")
    maude.load('ros_logical.maude')
    # m = maude.downModule(module_up)

    init = m.parseTerm("init <ROS2-Logical>")
    steps = init.rewrite()
    print(f'rew [{steps}] results in:', init)
    
from threading import Lock

class ThreadSafeDict:
    def __init__(self):
        self._dict = {}
        self._lock = Lock()

    def get(self, key):
        with self._lock:
            return self._dict.get(key)

    def set(self, key, value):
        with self._lock:
            self._dict[key] = value

    def delete(self, key):
        with self._lock:
            del self._dict[key]

    def reg(self,obj) -> int:
        i = id(obj)
        self.set(i,obj)
        print('reged',i,obj)
        return i

    def getByid(self,id) -> int:
        obj = self.get(id)
        # self.delete(id)
        # print(self._dict)
        print('get',id,obj)
        return obj

    def getByptr(self,term:maude.Term) -> int:
        print('get',term)
        term.reduce()
        print('get',term)

        id,*_ = term.arguments()
        id = id.toInt()
        return self.getByid(id)

def run_external(file):
    maude.init()
    rclpy.init()
    _object_registry = ThreadSafeDict()
    reg = _object_registry.reg 
    get = _object_registry.getByptr
    manager = NodeManager(reg,get)
    valueManager = ValueManager(reg,get)
    msgManager = MessageManager(reg,get)
    maude.connectRlHook('roshook',manager)
    maude.connectEqHook('rosvaluehook',valueManager)
    maude.connectEqHook('rosmsghook',msgManager)
    maude.load(file)

    if (m := maude.getCurrentModule()) is None:
        print('Bad module.')
        exit(1)

    # header = f"'{str(m)}"
    # module_up = maude.getModule('META-LEVEL').parseTerm(f"upModule({header},false)")
    # maude.load('ros_external.maude')
    # m = maude.downModule(module_up)
    
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