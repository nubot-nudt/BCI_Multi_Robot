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
	'dump', 'load',
]

import numpy, os
import cPickle as pickle
import gzip

#############################################################

def dump(f, *pargs, **kwargs):
	"""
	Append key and value variables to a binary file using pickle.
	
	* Normal usage:  dump(f, **kwargs)

	If f is a filename, open a file handle in binary append mode
	(a gzip file if f ends with '.gz', otherwise an uncompressed
	file). Alternatively, if f is not a string, assume that it is
	a file handle, and simply use that. Then, use cPickle to append,
	to this file, the key followed by the value of each keyword
	argument.

	
	* Special usage: dump(f, 'flush')
	
	Closes the file and returns None.
	
	
	* Special variable 'discard':
	
	e.g.   dump(f, discard={'x':3, 'y':4})
	Flags that the previous 3 dumped x's and the previous 4 dumped
	y's should be skipped when the file is read.
	
	"""###
	if len(pargs):
		if len(pargs) > 1 or len(kwargs) > 0: raise TypeError("use only keyword arguments (exception: one single string, not followed by keyword args)")
		command = pargs[0]
		if not isinstance(command, basestring): raise TypeError("subcommand must be a string")
		if command == 'flush':
			if hasattr(f, 'close'): f.close()
			f = None
		else:
			raise ValueError("unrecognized subcommand '%s'" % command)
		return f	
	if isinstance(f, basestring):
		if f.lower().endswith('.gz'): openfn = gzip.open
		else: openfn = open
		f = openfn(f, 'ab')
	for k,v in kwargs.items():
		pickle.dump((k,v), f, protocol=2)
	return f
	
#############################################################

def juggledim(x, d):
	x = numpy.asarray(x)
	sh = list(x.shape)
	while len(sh) < d: sh.append(1)
	x.shape = tuple(sh)
	return numpy.expand_dims(x, d)

#############################################################

def start(f):
	if isinstance(f, basestring):
		fn = f
		if not os.path.isfile(f) and os.path.isfile(f + '.gz'): f += '.gz'
		if f.lower().endswith('.gz'): openfn = gzip.open
		else: openfn = open
		f = openfn(f, 'rb')
	elif hasattr(f, 'filename'):
		fn = f.filename
	elif hasattr(f, 'name'):
		fn = f.name
	else:
		fn = str(f)
	f.seek(0)
	return f, fn

#############################################################

def scan(f, catdim=0, discard=True):

	f,fn = start(f)
	print "scanning " + fn
	
	counts = {}
	indices = {}
	dims = {}
	dtypes = {}
	
	while 1:
		try: k,v = pickle.load(f)
		except EOFError: break
		
		if k == 'discard':
			if discard:
				for kk,vv in v.items():
					if vv < 1: continue
					indices[kk] = indices.get(kk, [])[:-vv]
			continue
		
		nseen = counts.get(k, 0)
		counts[k] = nseen + 1
		indices[k] = indices.get(k, [])
		indices[k].append(nseen)
		
		try: v = numpy.asarray(v)
		except: pass
		else:
			v = juggledim(v, catdim)
			if k not in dims:
				dims[k] = list(v.shape)
				dtypes[k] = v.dtype
			if tuple(v.shape) != tuple(dims[k]): raise ValueError("dimensions of variable %s are not consistent" % k)
				
	return {'file':f, 'catdim':catdim, 'indices':indices, 'dims':dims, 'dtypes':dtypes, }

#############################################################

def load(f, catdim=0, maxcount=None, discard=True):
	"""
	Load the contents of a file dumped by dump()
	"""###
		
	if not isinstance(f, (tuple,list)): f = [f]
	hdrs = [scan(fi, catdim=catdim, discard=discard) for fi in f]

	#for i,h in enumerate(hdrs):
	#	print "hdrs[%d]"%i
	#	for k,v in h['indices'].items(): print "%s: %d" %(k,len(v))
	totals = {}
	dims    = hdrs[0]['dims']
	dtypes  = hdrs[0]['dtypes']	
	for h in hdrs[1:]:
		for k,v in h['indices'].items():
			if len(v) == 1 and len(hdrs[0]['indices'][k]) == 1:
				h['indices'][k] = [] # if a variable appears only once in the current file, and also only once in the first file, ignore it in the current file
	#for i,h in enumerate(hdrs):
	#	print "hdrs[%d]"%i
	#	for k,v in h['indices'].items(): print "%s: %d" %(k,len(v))
	for h in hdrs:
		for k,v in h['indices'].items():
			sofar = totals.get(k, 0)
			if maxcount == None: trim = 0
			else: trim = max(0, sofar + len(v) - maxcount)
			if trim: h['indices'][k] = v = v[:-trim]
			totals[k] = sofar + len(v)
	
	# prepare
	#   need:   indices, dims, dtypes
	#   create: content
	
	content = {}
	for k,siz in sorted(dims.items()):
		n = totals[k]
		if n == 1:
			content[k] = None
		else:
			siz[catdim] = n
			print "initializing variable %s with size %s" % (k,siz)
			content[k] = numpy.zeros(siz, dtype=dtypes[k])
		
	# read
	#   need:   f, indices
	#   create: discards
	#   modify: content
	
	filled = {}
	discards = {}
	subs = {}
	
	for hdr in hdrs:
		
		indices = hdr['indices']
		dims    = hdr['dims']
		dtypes  = hdr['dtypes']
		f,fn = start(hdr['file'])
		
		print "loading from " + fn
		
		seen_in_this_file = {}
		accepted_from_this_file = {}
		
		while 1:
			try: k,v = pickle.load(f)
			except EOFError: break

			if k == 'discard': continue
				
			naccepted = accepted_from_this_file.get(k, 0)
			nseen = seen_in_this_file.get(k, 0)
			seen_in_this_file[k] = nseen + 1
			if naccepted < len(indices[k]): expected = indices[k][naccepted]
			else: expected = None
			if nseen != expected:
				discards[k] = discards.get(k, 0) + 1
				continue
			accepted_from_this_file[k] = naccepted + 1
			
			filled[k] = filled.get(k, 0)
			a = content[k]
			if a == None:
				content[k] = v
			elif filled[k] >= content[k].shape[catdim]:
				raise RuntimeError("this shouldn't happen")
			else:
				v = juggledim(v, catdim)
				if k not in subs: subs[k] = [slice(None) for s in v.shape]
				subs[k][catdim] = filled[k]
				if len(subs[k]) == 1: content[k][subs[k]] = v  # either a bug or a very weird feature in numpy requires this special case
				else:                 content[k][subs[k]].flat = v.flat # though this *should* do the job either way
			filled[k] += 1
			
	if len(discards):
		print "discarded: " + ', '.join([('%s:%d' % (k,v)) for k,v in sorted(discards.items())])
	
	return content

#############################################################

def dumpmethod(self, *pargs, **kwargs):
	"""
	Append key and value variables to a binary file using pickle.
	
	* Normal usage:  self.dump(**kwargs)

	If no dump file is yet open, open one in append mode,
	using a filename/path based on the current self.data_file.
	Then use cPickle to append, to this file, the key followed
	by the value of each keyword argument.

	* Special usage: self.dump('flush')
	
	Closes the file handle and forgets the filename (so that a
	new default filename can be generated next time around).

	"""###
	self._dumpfilename = getattr(self, '_dumpfilename', None)
	self._dumpfilehandle = getattr(self, '_dumpfilehandle', None)
	self._dumphistory = getattr(self, '_dumphistory', {})		

	if self._dumpfilename == None:
		if self.data_file == None or len(self.data_file) == 0: raise RuntimeError("self.data_file has not been assigned")
		self._dumpfilename = os.path.realpath(os.path.splitext(self.data_file)[0] + '.pk') # NB: using .pk.gz sometimes results in 'CRC check failed' exceptions while trying to load after a trigger-trap error occurred

	dh = self._dumphistory[self._dumpfilename] = self._dumphistory.get(self._dumpfilename, {})	
	
	arg = self._dumpfilehandle
	if arg == None: arg = self._dumpfilename
	arg = dump(arg, *pargs, **kwargs)
	self._dumpfilehandle = arg
	if arg == None: self._dumpfilename = None  # a returned None instead of a filehandle is the sign that the file has been closed

	for k in kwargs: dh[k] = dh.get(k, 0) + 1
		
#############################################################

def loadmethod(self, f=None, catdim=-1):
	if f == None:
		f = os.path.splitext(self.data_file)[0] + '.pk'
		if not os.path.isfile(f) and os.path.isfile(f + '.gz'): f += '.gz'
	elif isinstance(f, basestring) and not os.path.isfile(f) and len(os.path.split(f)[0])==0:
		f = os.path.join(self.data_dir, f)
	if isinstance(f, basestring): f = os.path.realpath(f)
	if f == getattr(self,'_dumpfilename'):
		try: self._dumpfilehandle.close()
		except: print "\nWARNING: failed to close current dump file: %s may still be open\n" % f
		else: self._dumpfilehandle = self._dumpfilename = None
	return load(f=f, catdim=catdim) 

#############################################################

try: from BCI2000PythonCore import BciCore
except ImportError: from BCPy2000.Generic import BciCore
BciCore.load = loadmethod
BciCore.dump = dumpmethod
