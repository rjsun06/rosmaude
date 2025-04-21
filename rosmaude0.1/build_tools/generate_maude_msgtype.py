import os
import argparse
import ros2interface.verb.show as show
import basetype

basetype2sort = {
    'string' : "String",
    'int32' : 'Int',
    'uint32' : 'Int',
}

def to_sort(type):
    global type2sort
    global basetype2sort
    if type in type2sort:
        return type2sort[type]
    else:
        if str(type) not in basetype2sort:
            print(str(type).__class__)
            print(str(type),'is not a basetype')
            exit(1)
        return basetype2sort[str(type)]
    
def a(type):
    name = to_sort(type)
    out=f"""\
fmod {name} is ex ROS2-DATA
\tsorts {name} .
\tsubsort {name} < Data
\tsorts {name}-Attribute {name}-AttributeList.
\tsubsort {name}-Attribute < {name}-AttributeList.
\top {name} : -> Ros2-DataType .
\top _,_ : {name}-AttributeList {name}-AttributeList -> {name}-AttributeList [ctor comm assoc].
\top {name}<_> : {name}-AttributeList -> {name} .
    """
    dep = ""
    for line in show._get_interface_lines(type):
        if not line._field: continue
        key = line._field.name
        sort = to_sort(line._field.type)
        if sort not in basetype2sort:
            dep += f"sload {sort}.maude\n"
        out += f"\top {key}:_ : {sort} -> {name}-Attribute\n"
    with open(f'{name}.maude','w') as f:
        f.write(dep)
        f.write(out)
        f.write("endfm")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='generate maude definition for a ros2 interface',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('mapping', type=str, 
                       help='maude sort to ros2 message mapping file')
    parser.add_argument('-d', '--directory', type=str,
                       help='output dir')
    
    args = parser.parse_args()

    global type2sort 
    type2sort = {}
    with open(args.mapping) as f:
        for line in f.readlines():
            type, sort = line.split(' ')
            type2sort[type] = sort
    if args.directory:
        os.chdir(args.directory)
    for k in type2sort:
        a(type)