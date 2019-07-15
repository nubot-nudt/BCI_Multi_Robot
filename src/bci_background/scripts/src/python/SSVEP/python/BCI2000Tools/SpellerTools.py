#   $Id$
#   
#   This file is part of the BCPy2000 framework, a Python framework for
#   implementing modules that run on top of the BCI2000 <http://bci2000.org/>
#   platform, for the purpose of realtime biosignal processing.
# 
#   Copyright (C) 2007-10  Jeremy Hill, Thomas Schreiner,
#                          Christian Puzicha, Jason Farquhar
#   
#   bcpy2000@bci2000.org
#   
#   The BCPy2000 framework is free software: you can redistribute it
#   and/or modify it under the terms of the GNU General Public License
#   as published by the Free Software Foundation, either version 3 of
#   the License, or (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
__all__ = [
	'CodebooksItem', 'CodebooksParam',
	'GridItem', 'GridPage', 'GridParam', 
]

import numpy

##############################################################################################
#### General-purpose
##############################################################################################

def MultilineStringToNumpyArray(string, dtype=None):
	if dtype == None: dtype = numpy.float64
	string = string.replace('.', '0').replace('-', '0')
	return numpy.array([x.lstrip('\t ').rstrip('\t ').split() for x in string.lstrip('\n\t ').rstrip('\n\t ').split('\n')], dtype=dtype)

##############################################################################################

def ConstructMatrixSubparam(matrix, fmt='%d'):
	return '{ matrix ' + str(matrix.shape[0]) + ' ' + str(matrix.shape[1]) + ' ' + \
		' '.join([' '.join([fmt % entry for entry in row]) for row in matrix]) + \
		' }'		

##############################################################################################
#### Codebooks parameter manipulation
##############################################################################################

CodebooksParamHeadings = (('Name', str), ('Pages', str), ('RandomizeTime', str), ('RandomizeSpace', str), ('Matrix', str))

##############################################################################################

def GetCode(listofcodes, key):
	c = [x for x in listofcodes if x['Name'] == key]
	if len(c) < 1: raise IndexError("Name='%s' is not found in codebook list" % key)
	if len(c) > 1: raise IndexError("Name='%s' is not unique in codebook list" % key)
	return c[0]

##############################################################################################

def ConstructRowOfCodebooksParam(c, pages=None, order=None, name=None):	
	if name == None: name = c['Name']
	if pages == None: pages = c['Pages']
	if isinstance(pages, (tuple,list,basestring)) and len(pages) == 0: pages = '%'
		
	m = c['Matrix']
	
	if not type(m) == numpy.ndarray:
		m = MultilineStringToNumpyArray(m, numpy.int)
	
	if order != None:
		if len(order) != m.shape[0]:
			raise IndexError('Wrong dimensions for reodering')
		m = m[order]
		
	if c['RandomizeTime'] not in [0, 1] and len(c['RandomizeTime']) != m.shape[1]:
		raise IndexError('RandomizeTime failed because the number of columns in codebook and parameter do not match')
		
	if c['RandomizeSpace'] not in [0, 1] and sum(c['RandomizeSpace']) != m.shape[0]:
		raise IndexError('RandomizeSpace failed because the number of rows in codebook and parameter do not match')
		
	mstr = ConstructMatrixSubparam(m) # m could be a space/newline-delimited string,  or an actual numpy array of numeric 1's and 0's
	
	return name.replace(' ', '%20') + ' ' + \
		str(pages).strip('[,]').replace(', ',',').replace(',',' ').replace(' ','%20') + ' ' + \
		str(c['RandomizeTime']).strip('[,]').replace(', ',',').replace(',',' ').replace(' ','%20') + ' ' + \
		str(c['RandomizeSpace']).strip('[,]').replace(', ',',').replace(',',' ').replace(' ','%20') + ' ' + mstr

##############################################################################################

def ConstructCodebooksParam(*coderows):
	if len(coderows) == 1 and isinstance(coderows[0], (tuple,list)): coderows = coderows[0]
	names = [x.partition(' ')[0] for x in coderows]
	if len(set(names)) < len(names): raise ValueError("duplicate names in codebook")
	return 'EncDec:Encoding matrix Codebooks= ' + str(len(coderows)) + ' { ' +  ' '.join([z[0] for z in CodebooksParamHeadings])  + ' } ' + ' '.join(coderows)

##############################################################################################

import copy
from SigTools.NumTools import sstruct

class CodebooksItem(sstruct):
	def __init__(self, d, pages=1, reorder=None):
		if not isinstance(d, dict): d = d.__dict__
		sstruct.__init__(self, d)
		if isinstance(self.Matrix, str): self.Matrix = MultilineStringToNumpyArray(self.Matrix)
		self.Pages = pages
		self._reorder_fields(('Name', 'Pages', 'RandomizeTime', 'RandomizeSpace', 'Matrix'), 0)
		if reorder != None: self.reorder(reorder)
		self._maxstrlen = 1000
	def __str__(self):
		return str(CodebooksParam([self]))
	def __add__(self, other):
		return CodebooksParam(self) + other
	def __setitem__(self, key, val):
		return setattr(self, key, val)
	def __getitem__(self, key):
		return getattr(self, key)
	def write(self, *pargs, **kwargs):
		return CodebooksParam(self).write(*pargs,**kwargs)
	def set(self, key, val):
		setattr(self, key, val)
		return self
	def reorder(self, order):
		self.Matrix = self.Matrix[order]
		return self

##############################################################################################

class CodebooksParam(list):
	def __init__(self, arg):
		if not isinstance(arg, (tuple,list)): arg = [arg]
		list.__init__(self, arg)
		for i in range(len(self)):
			x = self[i]
			#x = copy.deepcopy(x)
			if not isinstance(x, CodebooksItem): x = CodebooksItem(x)
			self[i] = x		
	def __delitem__(self, ind):
		if isinstance(ind, str): ind = [x.Name for x in self].index(ind)
		return list.__delitem__(self, ind)	
	def __getitem__(self, ind):
		if isinstance(ind, str): return GetCode(self, ind)
		else: return list.__getitem__(self, ind)	
	def __add__(self, other):
		if not isinstance(other, (list,tuple)): other = [other]
		return self.__class__(list.__add__(self,other))	
	def __str__(self):
		return ConstructCodebooksParam([ConstructRowOfCodebooksParam(x) for x in self])
	def write(self, filename, append=False):
		open(filename, mode={True:'a', False:'w'}[append]).write(str(self) + '\n')
	def set(self, key, val):
		for x in self: x.set(key, val)
		return self
	def reorder(self, order):
		for x in self: x.reorder(order)

##############################################################################################
#### Grid parameter manipulation
##############################################################################################

GridParamHeadings = (("Display",str), ("Scale",float), ("Row",float), ("Column",float), ("Page",int), ("Action",str))

##############################################################################################

def ReorderSubsetOfGrid(grid, page, reordering):
	gridEntries = [x for x in grid[grid.find('}')+1:].split(' ') if not len(x)==0]
	colHeader = cols = [x for x in grid[grid.find('{')+1:grid.find('}')].split(' ') if not len(x)== 0]
	numRows = int(grid[grid.find('=')+1:grid.find('{')])

	gridArray = [[]]*numRows	
	reorderPage = [[]]*len(reordering)
	k=0
	for i in range(numRows):
		gridArray[i] = [[]]*len(colHeader)
		for j in range(len(colHeader)):
			gridArray[i][j] = gridEntries[j+ i*len(colHeader)]
		if int(gridArray[i][4]) == page:
			reorderPage[k] = gridArray[i]
			k = k + 1   	

	if len(reordering) != len([x for x in gridArray if int(x[4]) == page]):
		raise Exception('Your reordering parameter does not match the page in length')
	if len([x for x in reordering if x >= len(reordering) or x < 0]) > 0:
		raise Exception('Your reordering parameter contains invalid entries')
		
	reorderedGrid = [[]]*numRows
	j = 0
	for i in range(numRows):
		if int(gridArray[i][4]) == page:
			reorderedGrid[i] = ' '.join(reorderPage[reordering[j]])
			j = j + 1
		else:
			reorderedGrid[i] = ' '.join(gridArray[i])
		
	return grid[0:grid.find('}')+2] + ' '.join(reorderedGrid)

##############################################################################################

def ReadGridFile(fn):
	import codecs
	txt = open(fn).read()
	txt = txt.decode('utf-8').lstrip(unicode(codecs.BOM_UTF8, 'utf-8'))
	txt = txt.replace('\r\n', '\n').replace('\r', '\n')
	return [x.lstrip('\t') for x in txt.split('\n')]

##############################################################################################

def MakeGrid(arg, dtype=None):
	if isinstance(arg, str):
		if '\n' in arg:
			#multiline string
			grid = ParseGrid(arg.split('\n'))
		else:
			#filename
			arg = ReadGridFile(arg)
			grid = ParseGrid(arg)
	
	elif isinstance(arg, (list, tuple)):
		#list of string pages
		if isinstance(arg[0], str):
			grid = [ParseGridPage(x) for x in arg]
		
		if isinstance(arg[0], (tuple, list)):	
			#list of list of string lines
			if isinstance(arg[0][0], str):
				grid = [[ParseGridLine(line) for line in page] for page in arg]
				
	else:
		raise Exception('No suitable conversion known')
	
	if dtype == GridPage and len(grid)==1:
		return GridPage(grid[0])
	else:
		return GridParam(grid)	
	
##############################################################################################

def ParseGridLine(line, posY=1):
	if len(line) == 0: raise ValueError('Empty line')
	
	currTag = ''
	currPos = 0
	inTag = False
	line = line+' '
	tags = []
	for i in range(len(line)):
		if not line[i].isspace():
			inTag = True
			currTag = currTag + line[i]
			currPos = currPos + .5
		elif inTag:
			inTag = False
			# print repr(currTag), len(currTag) # TODO:  with a unicode character this is correct, but somehow the spacing afterwards gets thrown off 
			adjust = (len(currTag)%2 - 1)/2.0
			tags = tags + [GridItem(currTag, currPos + adjust, posY, '')]
			currPos = i+1
			currTag = ''
		else:
			currPos = currPos + 1;
	return tags

##############################################################################################

def ParseGridPage(page):
	lines = [ParseGridLine(page[i], i) for i in range(len(page))]
	ret = []
	for line in lines:
		ret = ret + line
	return ret
	
##############################################################################################

def ParseGrid(grid):
	gridStruct = []
	line = 0
	while True:
		try:
			i = grid.index('')
			p = ParseGridPage(grid[:i])
			if len(p): gridStruct.append(p)
			grid = grid[i+1:]
			line = line+1
		except ValueError:  # @@@ ValueError is too generic to be relied upon---can happen for all sorts of reasons---explicitly throw your own class of Exception if you need to (see TrieError in trie.py)
			gridStruct.append(ParseGridPage(grid))
			break
	return gridStruct

##############################################################################################

def escape(s):
	if len(s) == 0: return '%'
	out = ''
	for c in s:
		v = ord(c)
		if c == '%' or not 32 < v < 127: out += '%%%02x' % v
		else: out += str(c)
	return out
	
##############################################################################################

class GridItem:
	def __init__(self, label, posX, posY, command='', scale=None):
		self.label = label
		self.posX = posX
		self.posY = posY
		self.command = command
		if scale == None: scale = 0.1 * round(10.0 / len(label) ** 0.5)
		self.scale = scale
		
	def set(self, attrname, val):
		setattr(self, attrname, val)
	
	def __str__(self):
		s = "<%s.%s instance at 0x%08X>" % (self.__class__.__module__,self.__class__.__name__,id(self))
		label = unicode(self.label).encode('unicode_escape')
		s = s[:-1] + ' : %s  %2.1f  (% 5.1f,% 5.1f) %s >' % (label.rjust(5), self.scale, self.posY, self.posX, self.command)
		return s
				
	def __repr__(self):
		return str(self)

##############################################################################################

class GridPage(list):
	def __init__(self, arg):
		if len([1 for x in arg if not isinstance(x, GridItem)]):
			raise ValueError('Invalid Item - only GridItem objects are allowed')
		list.__init__(self, arg)
	
	def stringRep(self, pagenum, xtransform=None, ytransform=None):
		if xtransform == None: xmult, xadd = 1.0, 0.0
		else: xmult, xadd = xtransform
		if ytransform == None: ymult, yadd = 1.0, 0.0
		else: ymult, yadd = ytransform
		
		return '   '.join([
			'%s %s %g %g %s %s' % (escape(x.label), str(x.scale), x.posY*ymult+yadd, x.posX*xmult+xadd, str(pagenum), escape(x.command))
			for x in self])

	def __str__(self):
		return str(GridParam([self]))
		
	def __repr__(self):
		s = "<%s.%s instance at 0x%08X>" % (self.__class__.__module__,self.__class__.__name__,id(self))
		s += ' (%d items): [\n    ' % len(self) + ',\n    '.join([unicode(x).encode('raw_unicode_escape') for x in self]) + '\n]'
		return s
		
	def __getitem__(self, ind):
		if isinstance(ind, int):
			return list.__getitem__(self, ind)
		else:
			return list.__getitem__(self, self.index(ind))
		
	def index(self, label):
		if isinstance(label, int):
			if label >= len(self):
				raise ValueError('Index too large')
			else: return label
		else:
			ind = [i for i in range(len(self)) if self[i].label == label]
			if len(ind) > 0: return ind[0]
			else: raise ValueError('Label not found')
			
	def set(self, label, attrname, val):
		self[label].set(attrname, val)
	
	def reorder(self, order, position=0):
		if isinstance(order, (int, basestring)): order = [order]
		move = [self[i] for i in order]
		for x in move:
			self.remove(x)
			self.insert(position, x)
			position += 1

	def write(self, *pargs, **kwargs):
		return GridParam(self).write(*pargs,**kwargs)

##############################################################################################

class GridParam(list):
	def __init__(self, arg):
		if isinstance(arg, str):
			for x in MakeGrid(arg):
				if len(x): self.append(x)
			return
			
		if isinstance(arg, GridPage) or not isinstance(arg, (tuple,list)): arg = [arg]
		list.__init__(self, arg)
		for i in range(len(self)):
			x = self[i]
			if not isinstance(x, GridPage): x = GridPage(x)
			self[i] = x		
			
	def __add__(self, other):
		if not isinstance(other, (list,tuple)): other = [other]
		return self.__class__(list.__add__(self,other))	

	def __str__(self):
		x,y = numpy.array([(item.posX,item.posY) for item in reduce(list.__add__, self)]).T
		x,y = numpy.unique(x), numpy.unique(y)
		xtransform = (len(x)-1.0)/x.ptp(),  1 - (len(x)-1.0)*x.min()/x.ptp()
		ytransform = (len(y)-1.0)/y.ptp(),  1 - (len(y)-1.0)*y.min()/y.ptp()
				
		intro = 'PythonApp:Stimulus matrix Grid= ' + str(sum([len(x) for x in self])) + ' { '
		paramHeadings = ' '.join([escape(x) for x,y in GridParamHeadings])
		params = ' } ' + '   '.join([self[i].stringRep(i+1, xtransform, ytransform) for i in range(len(self))])
		return intro + paramHeadings + params
	
	def __repr__(self):
		s = "<%s.%s instance at 0x%08X>" % (self.__class__.__module__,self.__class__.__name__,id(self))
		s += ' (%d pages): [\n  ' % len(self) + ',\n  '.join([repr(x).replace('\n', '\n  ') for x in self]) + '\n]'
		return s
	
	def index(self, label, pagelist=[]):
		if isinstance(pagelist, int):
			pagelist = [pagelist]
		elif len(pagelist) == 0:
			pagelist = range(len(self))

		item = None			
		for x in pagelist:
			try:
				item = self[x].index(label)
				break
			except ValueError:
				pass
		if not item == None:
			return (x,item)
		raise ValueError('Item not found')

	def set(self, label, attrname, val, page=[]):
		ind = self.index(label, page)
		self[ind[0]].set(label, attrname, val)

	def write(self, filename, append=False):
		open(filename, mode={True:'a', False:'w'}[append]).write(str(self) + '\n')
	
##############################################################################################
##############################################################################################
