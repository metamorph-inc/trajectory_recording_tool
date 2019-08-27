#!/usr/bin/env python
# -*- coding: utf-8 -*

import re


def main():
    example_str = '{\n    ' \
               '"times": [\n        0.0,\n        1.0\n    ],\n    ' \
               '"waypoints": {\n        ' \
               '"0": {\n            ' \
               '"accelerations": [\n                NaN,\n                NaN,\n                NaN\n            ],\n            ' \
               '"names": [\n                "HEBI/Base",\n                "HEBI/Shoulder",\n                "HEBI/Elbow"\n            ],\n            ' \
               '"positions": [\n                0.08157739043235779,\n                -0.16892193257808685,\n                -0.22417549788951874\n            ],\n            ' \
               '"velocities": [\n                NaN,\n                NaN,\n                NaN\n            ]\n        },\n        ' \
               '"1": {\n            ' \
               '"accelerations": [\n                NaN,\n                NaN,\n                NaN\n            ],\n            ' \
               '"names": [\n                "HEBI/Base",\n                "HEBI/Shoulder",\n                "HEBI/Elbow"\n            ],\n            ' \
               '"positions": [\n                0.08163727074861526,\n                -0.1690678596496582,\n                -0.22420595586299896\n            ],\n            ' \
               '"velocities": [\n                NaN,\n                NaN,\n                NaN\n            ]\n        }\n    }\n}'

    def newlinereplace(matchobj):
        no_newlines = matchobj.group(2).replace("\n","")  # eliminate newline characters
        no_newlines = no_newlines.split()  # eliminate excess whitespace
        no_newlines = "".join([" " + segment.strip() for segment in no_newlines])
        return matchobj.group(1) + no_newlines

    match_re = r'("[a-z]+":\s\[)((\n\s*"*[\w\./\-\d]+"*,)+\n\s*("*[\w\./\-\d]+)"*\n\s*])'
    sub_str = re.sub(match_re, newlinereplace, example_str)
    print(sub_str)


if __name__ == '__main__':
    main()