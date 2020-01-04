# BCI common functions
# contributors: Liu Yang (gloolar@gmail.com)

__all__ = [
	'toRGB', 'create_logger', 'generate_cube_codebook', 'generate_RC_codebook', 'generate_extended_RC_codebook'
]

##################################################################

def toRGB(hex_color_str):
	"""
	transform hex color string to integer tuple.
	e.g. r,g,b = toRGB('0xFFFFFF')
	"""
	
	return int(hex_color_str[2:4],16)/255., int(hex_color_str[4:6],16)/255., int(hex_color_str[6:8],16)/255.

#################################################################

def create_logger(exp_name, subject_name, datadir_name, session):
	import logging, os
	
	logger = logging.getLogger(exp_name)
	if len(logger.handlers) == 0:
		logger.setLevel(logging.DEBUG)
		
		loggerfn = os.path.join(os.curdir, datadir_name, subject_name+session, subject_name+'S'+session+'_summary.log')
		fh = logging.FileHandler(loggerfn)
		fh.setLevel(logging.INFO)
		formatter = logging.Formatter("%(asctime)s - %(name)s - %(message)s")
		fh.setFormatter(formatter)
		
		ch = logging.StreamHandler()
		ch.setLevel(logging.DEBUG)
		formatter = logging.Formatter("%(message)s")
		ch.setFormatter(formatter)
		
		logger.addHandler(fh)
		logger.addHandler(ch)
	
	return logger
	
#################################################################

#################################################################
def generate_cube_codebook(cube_dim, reps, permutate=1, item_num=0):
	"""
	generate cube stimulus codebook for one selection sequence
	"""
	
	import numpy as np
	from random import choice
	
	codebook = []
	codeindex = []
	
	cube_size = np.prod(cube_dim)
	if item_num == 0 or item_num > cube_size:
		ntargets = cube_size
	else:
		ntargets = item_num
	
	if permutate:
		indices = np.random.permutation(cube_size)
	else:
		indices = np.arange(cube_size)
		
	cube = indices.reshape(cube_dim)
	
	last_dim = choice(range(len(cube_dim)))
	last_coord = choice(range(cube_dim[last_dim]))
	
	for rep in range(reps):

		# for new repetition, begin by last dimension of last repetition to avoid cross adjacency, then randomly permute the other dimensions
		dim_set0 = range(len(cube_dim))
		dim_set0.remove(last_dim)
		dim_set = [last_dim] + np.random.permutation(dim_set0).tolist()
	#	print "dim_set:",dim_set
	#	dim_set = np.random.permutation(range(len(self.cube_dim))) # simple random permutation
	#	dim_set = range(len(self.cube_dim)) # no random permutation
		last_dim = dim_set[-1]
		
		for d in dim_set:
	#		print "d:",d," index(d):",dim_set.index(d)
			# insert empty code before each dimension except the first to avoid cross adjacency
			if dim_set.index(d) > 0:
				codebook.append([])
				codeindex.append(-1)
			if cube_dim[d] > 1:
			
				if dim_set.index(d)==0: # for first dimension, avoid adjacency of the last coordinate of last repetition
					first_coord = choice(range(last_coord) + range(last_coord+1,cube_dim[d]))
	#				print "range(last_coord)+range(last_coord+1,cube_dim[d]):",range(last_coord)+range(last_coord+1,cube_dim[d])
					coord_set0 = range(first_coord) + range(first_coord+1,cube_dim[d])
					coord_set = [first_coord] + np.random.permutation(coord_set0).tolist()
	#				print "coord_set:",coord_set
				else:
					coord_set = np.random.permutation(range(cube_dim[d]))
			else:
				coord_set = [0]
		#	coord_set = range(self.cube_dim[d]) # no random permutation
			last_coord = coord_set[-1]
	#		print cube
			if len(coord_set) > 1:
				for i in coord_set:
					code_slice = cube.take([i],axis=d).flatten()
		#			print "code_slice",i,d,code_slice
					code_slice = code_slice[np.nonzero(code_slice<ntargets)]
					codebook.append(code_slice.tolist())
					codeindex.append(sum(cube_dim[:d])+i)
			#else:
			#	codebook.append([])
			#	codeindex.append([])				

	return cube, codebook, codeindex
	
#################################################################	
def generate_RC_codebook(cube_dim, reps, permutate=1, item_num=0):
	"""
	generate classic RC_mix codebook for one selection sequence
	"""

	import numpy as np
	
	codebook = []
	codeindex = []
	
	cube_size = np.prod(cube_dim)
	if item_num == 0 or item_num > cube_size:
		ntargets = cube_size
	else:
		ntargets = item_num
	
	indices = np.arange(cube_size)
	cube = indices.reshape(cube_dim)
	
	len_codebook = sum(cube_dim)#6col + 6row =12
	
	for rep in range(reps):
		# random permute the codebook for one rep
		ind_code = np.random.permutation(len_codebook)
		
		# avoid consecutive appearance between reps
		if rep > 0:
			while ind_code[0]==last_ind_code:
				ind_code = np.random.permutation(len_codebook)
		last_ind_code = ind_code[-1]
		
		for i in ind_code:
			if i < cube_dim[0]:
				code_slice = cube[i,:]
				code_slice = code_slice[np.nonzero(code_slice<ntargets)]
				codebook.append(code_slice.tolist())
			else:
				code_slice = cube[:,i-cube_dim[0]]
				code_slice = code_slice[np.nonzero(code_slice<ntargets)]
				codebook.append(code_slice.tolist())
		codeindex.extend(ind_code)	

	return cube, codebook, codeindex
	
#################################################################

def generate_extended_RC_codebook(cube_dim, reps, permutate=1, item_num=0):

	cube, codebook, codeindex = generate_RC_codebook(cube_dim, reps, permutate, item_num)

	for ci in range(0,len(codeindex)):	
		if codeindex[ci] in range(0,cube_dim[0]/2-1):
			for n in range(codeindex[ci]+1,cube_dim[1]/2):
				codebook[ci] = codebook[ci] + codebook[codeindex.index(n)]
				print codebook[n]
				print codebook[codeindex.index(n)]
		elif codeindex[ci] in range(cube_dim[0]/2+1,cube_dim[1]):
			for n in range(cube_dim[1]/2, codeindex[ci]):
				codebook[ci] = codebook[ci] + codebook[codeindex.index(n)]
				print codebook[n]
				print codebook[codeindex.index(n)]	
	return cube, codebook, codeindex