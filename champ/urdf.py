from urdf_parser_py.urdf import URDF


def _get_joint_origin(robot, joint_name):
    for joint in robot.joints:
        if joint.name == joint_name:
            xyz = [0,0,0]
            rpy = [0,0,0]

            try:
                xyz = joint.origin.xyz
            except:
                pass
            try:
                rpy = joint.origin.rpy
            except:
                pass
            return xyz, rpy

def _get_transform(robot, chain, ref_link, end_link):
    if ref_link == robot.get_root():
        start_index = 0
    else:
        start_joint = _get_attached_joint(robot, ref_link)
        start_index = chain.index(start_joint) + 1

    end_joint = _get_attached_joint(robot, end_link)
    end_index = chain.index(end_joint)

    x, y, z = 0, 0, 0
    r, p, y = 0, 0, 0

    for i in range(start_index, end_index + 1):
        pos, orientation = _get_joint_origin(robot, chain[i])
        if orientation == None:
            orientation = [0,0,0]
        
        if pos == None:
            pos = [0,0,0]
            
        x += pos[0]
        y += pos[1]
        z += pos[2]

    return x, y, z

def _get_attached_joint(robot, link_name):
    attached_joint = ""
    for joint in robot.joints:
        if joint.child == link_name:
            attached_joint = joint.name

    return attached_joint

def _get_joint_chain(robot, end_link):
    return robot.get_chain(robot.get_root(), end_link, links=False)

def get_translations(link_names, urdf_file=None):
    if urdf_file is None:
        robot = URDF.from_parameter_server()
    else:
        f = open(urdf_file, "rb")
        robot = URDF.from_xml_string(f.read())
    
    translations = []
    base = robot.get_root()
    
    for i in range(4):
        base_idx = i * 4
        joint_chain = _get_joint_chain(robot, link_names[base_idx + 3])
        translations.append(_get_transform(robot, joint_chain,  base, link_names[base_idx]))
        translations.append(_get_transform(robot, joint_chain,  link_names[base_idx], link_names[base_idx+1]))
        translations.append(_get_transform(robot, joint_chain,  link_names[base_idx+1], link_names[base_idx+2]))
        translations.append(_get_transform(robot, joint_chain,  link_names[base_idx+2], link_names[base_idx+3]))

    return translations



