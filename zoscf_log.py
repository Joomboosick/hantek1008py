"""
Example zero offset shift compensation function file
"""
import math

#  extracted from ch1-ch8_0V_24C-25.5C.csv.xz.s440.csv
#                    a       ,        b       ,     c
__zos_data = [
             [16.920415552899, -0.000381453840, 2012.6568],
             [21.070626029182, -0.000514977995, 1992.0672],
             [17.472230375023, -0.000398079698, 2008.5214],
             [19.685232121568, -0.000442328011, 2001.6491],
             [15.967868298571, -0.000336029388, 2010.2883],
             [16.050962032621, -0.000334699999, 2009.1168],
             [16.526705006909, -0.000337116893, 2002.3118],
             [18.756603380710, -0.000434281825, 2011.1874],
             ]
__zero_offset_start = [2037.0, 2023.0, 2032.0, 2030.0, 2034.0, 2033.0, 2026.0, 2039.0]


def calc_zos(ch: int, vscale: float, dtime: float) -> float:
    assert vscale == 1.0

    def exp(x, a, b, c):
        return a * math.e**(b * x) + c

    return __zero_offset_start[ch] - exp(dtime, *__zos_data[ch])


if __name__ == '__main__':
    print(*([t, calc_zos(0, 1.0, t)] for t in range(0, 120, 5)), sep="\n")
