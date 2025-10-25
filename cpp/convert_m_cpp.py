import json
import os

ml_vars: dict[str, str] = {}
has_been_init = set()

def get_cpp_datatype(shortcode: str):
    if shortcode == "1x1":
        return "double"
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

def convert_line(line: str):
    line = line.replace('%', '//')
    if line.strip().startswith('//'):
        return line
    line = line.replace("'", '.transpose()')

    if line.strip().startswith('if'):
        line = line + ' {'
    if line.strip() == 'end':
        line = '}'
    if line.strip().startswith('function'):
        return conv_func_header(line)
    
    if '=' in line:
        first_var = line.split('=')[0].strip()
        if first_var in ml_vars and first_var not in has_been_init:
            line = get_cpp_datatype(ml_vars[first_var]) + " " + line
            has_been_init.add(first_var)

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