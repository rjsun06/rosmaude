
#%%
import maude
import std_msgs.msg as msg

msg.Int16.get_fields_and_field_types()
#%%


def pack_baseType(base:maude.Term):
    symbol = str(base.symbol())
    a, = base.arguments()

    if symbol == "string":
        ret = a.prettyPrint(0).strip('"')
    elif symbol in ("int16","int32"):
        ret = a.toInt()
    
    return ret

def unpack_baseType(type:maude.Sort,data):
    symbol = type.
    a, = base.arguments()

    if symbol == "string":
        ret = a.prettyPrint(0).strip('"')
    elif symbol in ("int16","int32"):
        ret = a.toInt()
    
    return ret


def from_term(term : maude.Term):
    match str(term.symbol()):
        case 'str': return msg.String
        case 'int': return msg.Int32

def pack(msg_type,data:maude.Term):
    a, = data.arguments()
    # print(msg_type)
    m = data.symbol().getModule()
    print("upRaw",[msg_type.getSort().kind(), a.getSort().kind()],m.findSort("Raw").kind())

    data = m.findSymbol("upRaw",
                       [msg_type.getSort().kind(), data.getSort().kind()], 
                       m.findSort("Raw").kind())(msg_type,data)
    data.reduce()

    print(data)

    data = m.findSymbol("upRaw",
                       [a.getSort().kind()], 
                       m.findSort("Raw").kind())(a)
    data.reduce()
    print(data)

    assert(1==0)
    # if msg_type == msg._string.String:
    #     ret = msg.String()
    #     data, = data.arguments()
    #     ret.data = data.prettyPrint(0).strip('"')
    # assert ret is not None
    # return ret

def unpack(datatype,data):
    mod = datatype.symbol().getModule()
    if type(data) == msg._string.String:
        match str(datatype.symbol()):
            case 'str': return mod.parseTerm(f'["{str(data.data)}"]')
    assert 1==0 
    return
    
    
