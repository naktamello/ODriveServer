import json
import dpath
import os

from migrate_config.helper import recursive_json_iterator, getshape


cur_dir = os.path.dirname(os.path.abspath(__file__))
in_dir = os.path.join(cur_dir, 'in')
out_dir = os.path.join(cur_dir, 'out')

contents = [f for f in os.listdir(in_dir) if f.endswith('.json')]
with open('template.json') as infile:
    template = json.loads(infile.read())
template_shape = getshape(template)
template_keys = []
for k, v in recursive_json_iterator(template):
    template_keys.append(k)

for file_name in contents:
    old_config_path = os.path.join(in_dir, file_name)
    new_config_path = os.path.join(out_dir, file_name)
    with open(old_config_path) as infile:
        old_config = json.loads(infile.read())
    old_config_keys = []
    for k, v in recursive_json_iterator(old_config):
        old_config_keys.append(k)
    to_add = list(set(template_keys) - set(old_config_keys))
    to_remove = list(set(old_config_keys) - set(template_keys))
    new_config = dict()
    for key in old_config_keys:
        if key not in list(to_remove):
            dpath.util.new(new_config, key, dpath.util.values(old_config, key)[0])
    for key in list(to_add):
        dpath.util.new(new_config, key, dpath.util.values(template, key)[0])
    shape = getshape(new_config)
    assert template_shape == shape
    print("writing file:'{}'".format(file_name))
    with open(new_config_path, 'w') as outfile:
        outfile.write(json.dumps(new_config))
