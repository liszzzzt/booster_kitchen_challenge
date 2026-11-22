import math

action_dic = {
    "walk": "locomotion",
    "crouch": "locomotion",
    "pick": "manipulation",
}

object_cls_dic = {
    "f0":"simpleTable",
    "f1":"cabinet",
    "f2":"bin",
    "f3":"dishwasher",
    "o0":"cube",
    "o1":"cup",
    "o2":"plate",
}

navigation_goals = {
    "simpleTable": [
        3.61,
        0.10,
        math.radians(0.0),
    ],
}


schedule = {
    # 0: "spin",
    1: "walk to simpleTable",
    2: "pick cup",
}
