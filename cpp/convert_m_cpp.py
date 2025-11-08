import json
import os
import re # regex

ml_vars: dict[str, str] = {}
has_been_init = set()
has_been_init_bk = set()

def get_cpp_datatype(shortcode: str):
    if shortcode == "1x1":
        return "float"
    if shortcode == "struct":
        return "struct"
    if shortcode.split('x')[1] == "1":
        return "Vector" + shortcode.split('x')[0]
    return "Matrix" + shortcode.split('x')[0] + "_" + shortcode.split('x')[1]



def conv_func_header(line: str):
    line = line.replace('function ', '')
    line = line.replace(',', ', ')
    for varname in line.split():
        varname = varname.removesuffix(',').removesuffix(')')
        if varname in ml_vars:
            data_type = get_cpp_datatype(ml_vars[varname])
            line = line.replace(varname, data_type + ' ' + varname)
    
    ret_name = line.split(' ')[1]
    line = line.replace(ret_name, '', 1)
    line = line.replace('=', '', 1)
    line += ' {'

    return line

def rep_segment(line):
    matches = re.finditer(r"([a-zA-Z_]+)\((\d+):(\d+)\)", line)
    for match in matches:
        var, start, end = match.groups()
        start = int(start)
        end = int(end)
        line = line.replace(match.group(), f"{var}.segment<{end - start + 1}>({start - 1})")

    return line

def rep_block(line):
    matches = re.finditer(r"([a-zA-Z_]+)\((\d):(\d),\s*(\d):(\d)\)", line)
    for match in matches:
        var, start1, end1, start2, end2 = match.groups()
        start1 = int(start1)
        end1 = int(end1)
        start2 = int(start2)
        end2 = int(end2)
        line = line.replace(match.group(), f"{var}.block<{end1 - start1 + 1}, {end2 - start2 + 1}>({start1 - 1}, {start2 - 1})")

    return line

def rep_eye(line):
    matches = re.finditer(r"eye\((\d+)\)", line)
    for match in matches:
        size, = match.groups()
        size = int(size)
        line = line.replace(match.group(), f"Matrix{size}_{size}::Identity()")

    return line

def rep_one_vec(line):
    matches = re.finditer(r"ones\((\d+),\s*1\)", line)
    for match in matches:
        size, = match.groups()
        size = int(size)
        line = line.replace(match.group(), f"Vector{size}::Ones()")

    return line

def rep_zero_vec(line):
    matches = re.finditer(r"zeros\((\d+),\s*1\)", line)
    for match in matches:
        size, = match.groups()
        size = int(size)
        line = line.replace(match.group(), f"Vector{size}::Zero()")

    return line

def rep_zero_matrix(line):
    matches = re.finditer(r"zeros\((\d+),\s*(\d+)\)", line)
    for match in matches:
        size1, size2 = match.groups()
        size1 = int(size1)
        size2 = int(size2)
        line = line.replace(match.group(), f"Matrix{size1}_{size2}::Zero()")

    return line

def rep_norm(line):
    matches = re.finditer(r"([a-zA-Z_]+)\s*=\s*[a-zA-Z_]+\s*/\s*norm\([a-zA-Z_]+\)", line)
    for match in matches:
        var, = match.groups()
        line = line.replace(match.group(), f"{var}.normalize()")

    return line
    
    
    
def convert_line(line: str):
    global has_been_init
    global has_been_init_bk

    line = line.replace('%', '//')
    if line.strip().startswith('//'):
        return line
    line = line.replace("'", '.transpose()')
    line = line.replace('expm', 'matrixExpPade6')
    line = rep_segment(line)
    line = rep_block(line)
    line = rep_eye(line)
    line = rep_one_vec(line)
    line = rep_zero_vec(line)
    line = rep_zero_matrix(line)
    line = rep_norm(line)

    if line.strip().startswith('if'):
        line = line + ' {'
        has_been_init_bk = has_been_init.copy()
    if line.strip() == 'end':
        line = '}'
        has_been_init = has_been_init_bk.copy()
    # if line.strip().startswith('function'):
    #     return conv_func_header(line)
    
    if '=' in line:
        first_var = line.split('=')[0].strip()
        if first_var in ml_vars and first_var not in has_been_init:
            line = get_cpp_datatype(ml_vars[first_var]) + " " + line.lstrip()
            has_been_init.add(first_var)

    # special replacements:
    line = line.replace("sum(lastZ.segment<9>(0) - z.segment<9>(0)) ~=0", "(new_imu_packet)")
    line = line.replace("sum(lastZ.segment<6>(9) - z.segment<6>(9)) ~=0", "(new_gps_packet)")

    return line


def convert_file(fname):
    global ml_vars
    with open(fname) as f:
        lines = f.readlines()

    with open('vars.json') as f:
        ml_vars = json.load(f)

    with open('auto_' + os.path.basename(fname).replace('.m', '.cpp'), 'w+') as f:
        for line in lines:
            f.write(convert_line(line.rstrip()) + '\n')


def main():
    fpath = r'State Estimation\Kalman FIlter\EstimateStateFCN.m'
    convert_file(os.path.dirname(__file__) + '/../' + fpath)

if __name__ == '__main__':
    main()