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
	'Param', 'escape',
]

import numpy

def escape(s):
	if isinstance(s, bool): s = int(s)
	if s == None: s = ''
	elif not isinstance(s, basestring): s = str(s)
	if len(s) == 0: return '%'
	out = ''
	for c in s:
		v = ord(c)
		if c == '%' or not 32 < v < 127: out += '%%%02x' % v
		else: out += str(c)
	return out
	

class Param(object):
	
	def __init__(self, x, tab='Tab', section='Section', name='ParamName', type='auto', comment='', fmt=None, range=(None,None), default=None, rlabels=None, clabels=None):
		self.tab = tab
		self.section = section
		self.type = type
		self.name = name
		self.rlabels = rlabels
		self.clabels = clabels
		self.x = x
		self.range = range
		self.default = default
		self.comment = comment
		self.verbosity = 1
	
	def determine_type(self):
		x = self.x
		if isinstance(x, numpy.ndarray):
			if len(x.shape) == 0: x = x.flat[0]
			elif len(x.shape) in (1,2): x = x.tolist()
			else: raise ValueError("don't know how to deal with >2-D arrays")		
		if isinstance(x, bool): return 'bool'
		if isinstance(x, int): return 'int'
		if isinstance(x, float): return 'float'
		if isinstance(x, basestring): return 'string'
		if isinstance(x, (tuple,list)):
			if False not in [isinstance(xi, int) for xi in x]: return 'intlist'
			if False not in [isinstance(xi, (int,float)) for xi in x]: return 'floatlist'
			if False not in [isinstance(xi, (int,float,basestring)) for xi in x]: return 'list'
			if False not in [isinstance(xi, (tuple,list)) for xi in x]: return 'matrix'
		raise ValueError("don't know how to deal with this data type")
	
	def make_string(self, verbosity=1):
		type = self.type
		comment = self.comment
		if verbosity < 1: comment = ''
		comment = ' // ' + comment
		if type in (None, 'auto'): type = self.determine_type()
		if type == 'bool': type = 'int'; comment = comment + ' (boolean)'
		s = '%s:%s %s %s= ' % (self.tab, self.section, type, self.name)		
		if type.endswith('list'):
			x = numpy.asarray(self.x).flat
			s += str(len(x))
			xstr = '    ' + ' '.join([escape(xi) for xi in x])
		elif type.endswith('matrix'):
			x = numpy.asarray(self.x)
			if self.rlabels == None: s += ' %d' % len(x)
			elif len(self.rlabels) != len(x):    raise ValueError("wrong number of row labels (got %d, expected %d)" % (len(self.rlabels), len(x)))
			else: s += ' { ' + ' '.join([escape(xi) for xi in self.rlabels]) + ' }' 
			if self.clabels == None: s += ' %d' % len(x[0])
			elif len(self.clabels) != len(x[0]): raise ValueError("wrong number of column labels (got %d, expected %d)" % (len(self.clabels), len(x[0])))
			else: s += ' { ' + ' '.join([escape(xi) for xi in self.clabels]) + ' }'
			xstr = '    ' + '    '.join([' '.join([escape(xi) for xi in row]) for row in x])
		else:
			xstr = escape(self.x)
		if verbosity == 0 and len(xstr) > 10: xstr = '...'
		s += ' ' + xstr
		if verbosity >= 2:
			range = self.range
			if range == None: range = (None,None)
			elif not isinstance(range, (tuple,list)): range = (0, range)
			s += '   ' + escape(self.default) + ' ' + escape(range[0]) + ' ' + escape(range[1])
		s += comment
		return s
	
	def writeto(self, file, append=False):
		if isinstance(file, basestring):
			mode = {True:'a', False:'w'}[append]
			file = open(file, mode + 't')
		if not append: file.seek(0)
		file.write(str(self)+'\n')
		
	def appendto(self, file):
		return self.writeto(file, append=True)		
	
	def __getslice__(self, s, e):
		return self.__getitem__(slice(s,e,None))

	def __getitem__(self, sub):
		def conv(self, i, x):
			if isinstance(x, (tuple,list)):
				if i == None: return [conv(self,i,xi) for i,xi in enumerate(x)]
				else: return [conv(self, i, xi) for xi in x]
			if i == None: i = 0
			if isinstance(x, slice): return slice(conv(self,i,x.start), conv(self,i,x.stop), conv(self,i,x.step))
			if isinstance(x, int):
				if x < 0:  x += numpy.asarray(self.x).shape[i]
				return x
			if not isinstance(x, basestring): return x
			if i == 0: lab = self.rlabels; labname = 'row'
			elif i == 1: lab = self.clabels; labname = 'column'
			else: raise TypeError("too many subscripts")
			if lab == None or x not in lab: raise ValueError("%s label '%s' not found" % (labname, x))
			return lab.index(x)
		sub = conv(self, None, sub)
		if not hasattr(sub, '__len__') or len(sub) == 1: return numpy.asarray(self.x).__getitem__(sub)
		elif len(sub) == 2: return numpy.asarray(numpy.asmatrix(self.x).__getitem__(sub))
		return numpy.asarray(self.x).__getitem__(sub)

	def __repr__(self):
		return '<%s object at 0x%08X>: %s' % (self.__class__.__name__, id(self), self.make_string(verbosity=0))
		
	def __str__(self):
		return self.make_string(verbosity=max(1, self.verbosity))
