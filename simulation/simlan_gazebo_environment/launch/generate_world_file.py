################## information ###################
# If you want to make more world configurations  #
#                 Add them here                  #
##################################################

import xml.etree.ElementTree as ET
import tempfile
def generate_world_file(world_setup, original_world):
    tree = ET.parse(original_world)
    root = tree.getroot()

    if world_setup == "medium":
        match_texts=['model://box_1185x785x1010']
        for parent in root.findall('.//*'):
            for include in list(parent):
                if include.tag == 'include':
                    uri = include.find('uri')
                    if uri is not None and any(text in uri.text for text in match_texts):
                        parent.remove(include)

    elif world_setup == "light":
        match_texts=['model://shelf', 'model://box_1185x785x1010', 'model://truck']
        for parent in root.findall('.//*'):
            for include in list(parent):
                if include.tag == 'include':
                    uri = include.find('uri')
                    if uri is not None and any(text in uri.text for text in match_texts):
                        parent.remove(include)

    elif world_setup == "empty":
        match_keywords = ['ground']
        for parent in root.findall('.//*'):
            for include in list(parent):
                if include.tag == 'include':
                    uri = include.find('uri')
                    if uri is not None and not any(keyword in uri.text for keyword in match_keywords):
                        parent.remove(include)


    temp_world = tempfile.NamedTemporaryFile(delete=False, suffix='.world')
    tree.write(temp_world.name)
    world=temp_world.name
    return world