import string
import os
import xml.etree.ElementTree as ET


def get_model_name_and_description(path_to_model_dir):
    tree = ET.parse(os.path.join(path_to_model_dir, 'model.config'))
    root = tree.getroot()
    name = root.findall('name')[0].text.strip()
    description = root.findall('description')[0].text.strip()
    return (name, description)


def models_in_dir(path_to_dir):
    for path in os.listdir(path_to_dir):
        abspath = os.path.abspath(os.path.join(path_to_dir, path))
        if os.path.isdir(abspath) and os.path.isfile(os.path.join(abspath, 'model.config')):
            yield abspath


if __name__ == '__main__':
    this_dir = os.path.abspath(os.path.dirname(__file__))
    models_dir = os.path.join(this_dir, '..', 'models')

    template_mapping = {
        'geometry': [],
        'materials': [],
        'joints': [],
        'links': [],
        'graphs': [],
        'poses': [],
        'models': []
    }

    for model_path in sorted(models_in_dir(models_dir)):
        name, desc = get_model_name_and_description(model_path)
        entry = f'* `{name}`\n  * {desc}'
        if name.startswith('geometry'):
            template_mapping['geometry'].append(entry)
        elif name.startswith('material'):
            template_mapping['materials'].append(entry)
        elif name.startswith('joint'):
            template_mapping['joints'].append(entry)
        elif name.startswith('link'):
            template_mapping['links'].append(entry)
        elif name.startswith('graph'):
            template_mapping['graphs'].append(entry)
        elif name.startswith('pose'):
            template_mapping['poses'].append(entry)
        elif name.startswith('model'):
            template_mapping['models'].append(entry)
        else:
            raise RuntimeError(f'Unknown model type {name} at {model_path}')

    for key in template_mapping.keys():
        template_mapping[key] = '\n'.join(template_mapping[key])

    with open(os.path.join(this_dir, 'README.md.in'), 'r') as fin:
        template = string.Template(fin.read())
        print(template.substitute(template_mapping))
