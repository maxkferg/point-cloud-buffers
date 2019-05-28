import re
import os
import sys

MODEL_REGEX = r"g[\d\s\w]+Model"
MTL_REGEX = r"usemtl [\d\s\w]"
MODEL_TEMPLATE = "g Mesh{0} Model"


def process_sdf(infile, outfile):
	"""
	Process an SDF file, line by line
	Insert MODEL_TEMPLATE before every new material
	Remove MODEL_REGEX (existing models) from file
	"""
	print("Reading obj file from %s"%infile)
	print("Writing obj file to %s"%outfile)
	with open(infile) as fdi, open(outfile, 'w') as fdo:
		model_num = 0
		for line in fdi:
			if re.search(MODEL_REGEX, line):
				continue
			if re.search(MTL_REGEX, line):
				model_line = MODEL_TEMPLATE.format(model_num)
				fdo.write(model_line)
				fdo.write("\n")
				model_num +=1
			fdo.write(line)


if __name__=="__main__":
	infile = "source/y2e2.obj"
	outfile = "pybullet/y2e2.obj"
	outdir = "pybullet"
	if not os.path.exists(outdir):
	    os.makedirs(outdir)
	process_sdf(infile, outfile)