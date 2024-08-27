import numpy as np

def normalize_vector(v): # ベクトルを正規化する関数
    norm = np.linalg.norm(v)
    if norm == 0: # ノルムがゼロのときは正規化できない
        return v
    return v / norm

def cross_three_dim(vector1, vector2): # 3次元の外積演算を行う関数
    result = np.array([
        [vector1[1]*vector2[2] - vector1[2]*vector2[1]],
        [vector1[2]*vector2[0] - vector1[0]*vector2[2]],
        [vector1[0]*vector2[1] - vector1[1]*vector2[0]]
    ])
    return result

def convert_data(matrix): # decodeにおけるファイル読み込みを容易にするためのに変換する関数
    list = matrix.tolist()
    str_list = ['[' + ', '.join(map(str, sublist)) + ']' for sublist in list]
    output = '\n'.join(str_list)
    return output

def colored_text(text, color_code): # ターミナルでのテキストの色を変える関数
    return f"\033[{color_code}m{text}\033[0m"