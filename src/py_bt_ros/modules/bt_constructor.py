import os
from modules.utils import (
    parse_behavior_tree,
    convert_value,
    get_file_dirname,
    optional_import,
)

def build_behavior_tree(agent, behavior_tree_xml: str, env_pkg: str):
    """
    Build a Behavior Tree from an XML file for the given agent and environment package.
    """
    bt_module = optional_import(f"{env_pkg}.bt_nodes")
    mission_bt_module = optional_import(f"{env_pkg}.mission_bt_nodes")

    if bt_module is None:
        raise ModuleNotFoundError(
            f"[ERROR] Could not import '{env_pkg}.bt_nodes'. "
            "Make sure your environment package exposes bt_nodes."
        )

    xml_root = parse_behavior_tree(behavior_tree_xml)
    return _parse_xml_to_bt(
        xml_root.find("BehaviorTree"),
        bt_module=bt_module,
        mission_bt_module=mission_bt_module,
        agent=agent,
        top_xml_path=behavior_tree_xml,
    )


def _parse_xml_to_bt(xml_node, *, bt_module, mission_bt_module, agent, top_xml_path):
    node_type = xml_node.tag

    # --- SubTree: inline from file ---
    if node_type == "SubTree":
        subtree_id = xml_node.attrib.get("ID")
        if not subtree_id:
            raise ValueError("[ERROR] SubTree node must have an 'ID' attribute")

        base_dir = get_file_dirname(top_xml_path)
        sub_behavior_tree_xml = os.path.join(base_dir, f"{subtree_id}.xml")
        subtree_root = parse_behavior_tree(sub_behavior_tree_xml)

        return _parse_xml_to_bt(
            subtree_root.find("BehaviorTree"),
            bt_module=bt_module,
            mission_bt_module=mission_bt_module,
            agent=agent,
            top_xml_path=sub_behavior_tree_xml,
        )

    # --- Regular node parsing ---
    children = [_parse_xml_to_bt(child,
                                 bt_module=bt_module,
                                 mission_bt_module=mission_bt_module,
                                 agent=agent,
                                 top_xml_path=top_xml_path) for child in xml_node]

    BTNodeList = getattr(bt_module, "BTNodeList")
    attrib = {k: convert_value(v) for k, v in xml_node.attrib.items()}

    # [핵심 수정] name 인자 충돌 방지 로직 추가
    # XML에 'name' 속성이 있으면 그걸 쓰고(pop), 없으면 태그 이름(node_type) 사용
    node_name = attrib.pop('name', node_type)

    if node_type in BTNodeList.CONTROL_NODES:
        control_class = getattr(bt_module, node_type)
        # name을 첫 번째 인자로 명시적으로 전달
        return control_class(node_name, children=children, **attrib)

    elif node_type in BTNodeList.DECORATOR_NODES:
        decorator_class = getattr(bt_module, node_type)
        if len(children) != 1:
            raise ValueError(f"[ERROR] Decorator '{node_type}' must have exactly 1 child.")
        return decorator_class(node_name, child=children[0], **attrib)

    elif node_type in (BTNodeList.ACTION_NODES + BTNodeList.CONDITION_NODES):
        action_class = getattr(bt_module, node_type)
        return action_class(node_name, agent, **attrib)

    elif node_type == "BehaviorTree":  # Root
        if not children:
            raise ValueError("[ERROR] <BehaviorTree> has no child node.")
        return children[0]

    else:
        raise ValueError(f"[ERROR] Unknown behavior node type: {node_type}")