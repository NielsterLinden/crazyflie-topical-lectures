"""Race course from the organisers' hoop surveys: start -> 3 -> 2 -> 1 -> 3 -> 2 -> 1 -> 3 (finish).
    PY race2/make_course.py <dir3>   dir3 = +1 (cross hoop 3 along +y) or -1
"""
import json, sys
import numpy as np

d3 = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
H = {i: json.load(open('race2/hoop-%d.json' % i)) for i in (1, 2, 3)}
dirs = {3: [0, d3, 0], 2: [1, 0, 0], 1: [0, 0, -1]}       # 2 crossed toward hoop 1 (+x), 1 crossed downward
hoops = []
for k, i in enumerate([3, 2, 1, 3, 2, 1, 3]):
    n = np.array(H[i]['normal']); n = n * np.sign(n @ dirs[i])
    hoops.append({'name': 'hoop-%d-pass%d' % (i, k + 1), 'centre': H[i]['centre'], 'normal': n.round(6).tolist(),
                  'radius_m': H[i]['radius_m']})
c3 = np.array(H[3]['centre'])
start = (c3 - np.array([0, d3 * 0.9, 0])).round(3).tolist()   # shortcut: assumed start 0.9 m before hoop 3, ask the real one
course = json.load(open('examples/course.synthetic.json'))
course.update(course_name='race2-hoops-321', start=start, finish=(c3 + np.array([0, d3 * 0.9, 0])).round(3).tolist(),
              hoops=hoops)
# shortcut: arena "about 3 x 5 m" around the hoops, orientation assumed (5 m along x); set to the real walls
course['limits'].update(bounds_min=[-1.6, -2.2, -0.2], bounds_max=[3.0, 1.6, 1.6])
out = 'race2/course_dir%+d.json' % d3
json.dump(course, open(out, 'w'), indent=1)
print(out)
