fpath = "AUG_JacobianX.m"
cpp_types = {
	'in1': ('Vector18', 18),
	'in2': ('Vector4', 4),
	'_return': ('Matrix18_18', 18 * 18)
}
# fpath = "AUG_plantfcn.m"
# cpp_types = {
# 	'in1': ('Vector18', 18),
# 	'in2': ('Vector4', 4),
# 	'_return': ('Vector18', 18)
# }
output_items = 0
return_varname = ''

def convert(line: str):
	global output_items
	global return_varname

	if return_varname and return_varname in line:
		return ""

	elif line.startswith('function'):
		name = line.split(' = ')[1].split('(')[0]
		return_varname = line.split(' ')[1]
		return f"{cpp_types['_return'][0]} {name}({cpp_types['in1'][0]} in1, {cpp_types['in2'][0]} in2) {{\n"
	
	elif line.startswith('%'):
		return f"//{line.removeprefix('%')}"

	elif line.startswith('end'):
		return f"return rval;\n}}\n"
	
	elif 'in1' in line or 'in2' in line: # b4 = in1(16,:);
		name = line.split(' = ')[0]
		input_name = line.split(' = ')[1].split('(')[0]
		idx = int(line.split('(')[1].split(',')[0])
		assert idx <= cpp_types[input_name][1]
		return f"float {name} = {input_name}[{idx - 1}];\n"

	elif line.strip() and '[' not in line: # basic expr
		line = line.replace('.*', '*').replace('./', '/')
		if '.^' in line: # t9 = q1.^2;
			name = line.split(' = ')[0]
			input_name = line.split(' = ')[1].split('.^')[0]
			pow_val = line.split('.^')[1].strip().removesuffix(';')
			line = f"{name} = pow({input_name}, {pow_val});\n"
			
		return f"float {line}"
	
	elif ' = [' in line:
		if output_items == 0:
			prefix = f"{cpp_types['_return'][0]} rval;\nrval << "
		else:
			prefix = ""

		line = line.replace('.*', '*').replace('./', '/')
		sep = ';' if ';' in line.rstrip().removesuffix(';') else ','
		line = line.split('[')[1].split(']')[0]
		output_items += len(line.split(sep))
		return prefix + line.replace(sep, ',\n') + ',\n'

	if line.strip():
		print(f"No conversion for {line}")
	return line



def main():
	with open(fpath) as f:
		lines = f.readlines()

	new_lines = []
	for line in lines:
		new_lines.append(convert(line))
	assert output_items == cpp_types['_return'][1]
	
	with open(fpath + '.cpp', 'w+') as f:
		f.writelines(new_lines)

if __name__ == '__main__':
	main()

	