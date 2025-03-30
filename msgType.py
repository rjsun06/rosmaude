
import maude
import std_msgs.msg as msg

class DataType:
    def getMsgType(self):
        pass

class maude_term:
    pass

def from_term(term : maude.Term):
    match str(term.symbol()):
        case 'str': return msg.String
        case 'int': return msg.Int32

def pack(msg_type,data:maude.Term):
    if msg_type == msg._string.String:
        ret = msg.String()
        data, = data.arguments()
        ret.data = data.prettyPrint(0).strip('"')
    assert ret is not None
    return ret

def unpack(datatype,data):
    mod = datatype.symbol().getModule()
    if type(data) == msg._string.String:
        match str(datatype.symbol()):
            case 'str': return mod.parseTerm(f'["{str(data.data)}"]')
    assert 1==0 
    return
    
    
