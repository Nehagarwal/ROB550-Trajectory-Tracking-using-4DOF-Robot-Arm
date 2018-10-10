# This library is provided under the GNU LGPL v3.0 license, with the 
# additional provision that any use in work leading to academic 
# publication requires at least one of the following conditions be met:
# (1) You credit all authors of the library, by name, in the publication
# (2) You reference at least one relevant paper by each of the authors
# 
# (C) Shai Revzen, U Penn. 2011

"""
File se3.py contains a library of functions for performing operations
in the classical Lie Group SE(3) and associated algebra se(3).

In the implementation, special care has been taken to provide vectorized
functions -- functions which operate in batch on multiple parameters.
In most cases, this takes the form of using the first indices of an
array and reusing the remaining indices, e.g. skew() applied to an array
of shape (4,7,3) would return result with shape (4,7,3,3), such that for
all i,j: result[i,j,:,:] = skew( parameter[i,j,:] )

Additionally, many functions have an _UNSAFE variant which skips all
validity checking on the inputs. These variants are provided for 
performance reasons. In general, USE THE SAFE FUNCTIONS unless you are
hitting a performance bottleneck and you are sure that the preconditions
for the computation are met.

NOTE to developers:
  This library uses python doctest to ensure that it performs as
  specified in the documentation. Please run with -v if you think
  something isn't working right, and send the failure case report
"""

from numpy import zeros, zeros_like, asarray, array, allclose, resize, ones, ones_like, empty, identity, prod, sqrt, isnan, pi, cos, sin, newaxis, diag, sum, arange, dot

from numpy.linalg import inv, eig

def skew( v ):
  """
  Convert a 3-vector to a skew matrix such that 
    dot(skew(x),y) = cross(x,y)
  
  The function is vectorized, such that:
  INPUT:
    v -- N... x 3 -- input vectors
  OUTPUT:
    N... x 3 x 3  
    
  For example:
  >>> skew([[1,2,3],[0,0,1]])
  array([[[ 0,  3, -2],
        [-3,  0,  1],
        [ 2, -1,  0]],
  <BLANKLINE>
       [[ 0,  1,  0],
        [-1,  0,  0],
        [ 0,  0,  0]]])
  """
  v = asarray(v).T
  z = zeros_like(v[0,...])
  return array([
      [ z, -v[2,...], v[1,...]],
      [v[2,...], z, -v[0,...] ],
      [-v[1,...], v[0,...], z ] ]).T

def unskew( S ):
  """
  Convert a skew matrix to a 3-vector
  
  The function is vectorized, such that:
  INPUT:
    S -- 3 x 3 x N... -- input skews
  OUTPUT:
    3 x N...  
  
  This is the "safe" function -- it tests for skewness first.
  Use unskew_UNSAFE(S) to skip this check
  
  Example:
  >>> x = array(range(24)).reshape(2,1,4,3); allclose(unskew(skew(x)),x)
  True
  >>> unskew([[1,2,3],[4,5,6],[7,8,9]])
  Traceback (most recent call last):
  ...
  AssertionError: S is skew
  """
  S = asarray(S)
  assert allclose(S.T.transpose((1,0)+tuple(range(2,S.ndim))),-S.T),"S is skew"
  return unskew_UNSAFE(S)

def unskew_UNSAFE(S):
  """
  Convert a skew matrix to a 3-vector
  
  The function is vectorized, such that:
  INPUT:
    S -- N... x 3 x 3 -- input skews
  OUTPUT:
    N... x 3  
  
  This is the "unsafe" function -- it does not test for skewness first.
  Use unskew(S) under normal circumstances
  """
  S = asarray(S).T
  return array([S[2,1,...],S[0,2,...],S[1,0,...]]).T

def screw( v ):
  """
  Convert a 6-vector to a screw matrix 
  
  The function is vectorized, such that:
  INPUT:
    v -- N... x 6 -- input vectors
  OUTPUT:
    N... x 4 x 4  
  """
  v = asarray(v)
  z = zeros_like(v[0,...])
  o = ones_like(v[0,...])
  return array([
      [ z, -v[...,2], v[...,1], v[...,3] ],
      [ v[...,2],  z,-v[...,0], v[...,4] ],
      [-v[...,1],  v[...,0], z, v[...,5] ],
      [ z,         z,        z, o] ])

def unscrew( S ):
  """
  Convert a screw matrix to a 6-vector
  
  The function is vectorized, such that:
  INPUT:
    S -- N... x 4 x 4 -- input screws
  OUTPUT:
    N... x 6
  
  This is the "safe" function -- it tests for screwness first.
  Use unscrew_UNSAFE(S) to skip this check
  """
  S = asarray(S)
  assert allclose(S[...,:3,:3].transpose(0,1),-S[...,:3,:3]),"S[...,:3,:3] is skew"
  assert allclose(S[...,3,:3],0),"Bottom row is 0"
  assert allclose(S[...,3,3],1),"Corner is 1"
  return unscrew_UNSAFE(S)

def unscrew_UNSAFE(S):
  """
  Convert a screw matrix to a 6-vector
  
  The function is vectorized, such that:
  INPUT:
    S -- N... x 4 x 4 -- input screws
  OUTPUT:
    N... x 6 
  
  This is the "unsafe" function -- it does not test for screwness first.
  Use unscrew(S) under normal circumstances
  """
  S = asarray(S)
  return array([S[...,1,2],S[...,2,0],S[...,0,1],
    S[...,0,3],S[...,1,3],S[...,2,3]])

def aDot( A, B ):
  """Similar to dot(...) but allows B.shape[-2] to equal A.shape[-1]-1
  for affine operations.
  
  Example:
  >>> aDot( seToSE([0,pi/2,0,1,1,1.0]), identity(3) )
  array([[ 1.,  1.,  0.],
       [ 1.,  2.,  1.],
       [ 2.,  1.,  1.]])
  """
  if B.shape[-2] != A.shape[-1]-1:
    return dot(A,B)
  C = dot( A[...,:-1,:-1],B )
  C += A[...,:-1,[-1]]
  return C

def aaToRot( aa ):
  """
  Convert skew vector into a rotation matrix using Rodregues' formula
  INPUT:
    aa -- N... x 3
  OUTPUT:
    N... x 3 x 3  
  
  >>> diag(aaToRot([3.1415926,0,0])).round(2)
  array([ 1., -1., -1.])
  >>> aaToRot([0,3.1415926/4,0]).round(2)
  array([[ 0.71,  0.  , -0.71],
         [ 0.  ,  1.  ,  0.  ],
         [ 0.71,  0.  ,  0.71]])
  """
  aa = asarray(aa)
  t = sqrt(sum(aa * aa,-1))
  k = aa / t[...,newaxis]
  k[isnan(k)]=0
  kkt = k[...,:,newaxis] * k[...,newaxis,:]
  I = identity(3)
  # Note: (a.T+b.T).T is not a+b -- index broadcasting is different
  R = (sin(t).T*skew(k).T + (cos(t)-1).T*(I-kkt).T).T + I
  return R

def seToSE( x ):
  """
  Convert a twist (a rigid velocity, element of se(3)) to a rigid
  motion (an element of SE(3))
  
  INPUT:
    x -- 6 x N...
  OUTPUT:
    3 x 3 x N...  

  !!!WARNING: VEC
  """
  x = asarray(x)
  R = aaToRot(x[:3,...])
  X = empty( (4,4)+x.shape[1:], dtype=x.dtype )
  X[:3,:3,...] = R
  X[3,:3,...] = 0
  X[3,3,...] = 1
  X[:3,3,...] = x[3:,...]
  return X
  
def Adj( K ):
  """
  The Adj operator for a rigid motion K
  
  This is the "safe" version which checks that K is a rigid motion
  
  !!!WARNING: VEC
  """
  K = asarray(K)
  sz = K.shape[2:]
  assert K.shape[:2] == (4,4)
  N = int(prod(sz))
  K.shape = (4,4,N)
  I = identity(3)
  for k in xrange(N):
    assert allclose(dot(K[:3,:3,k],K[:3,:3,k].T),I), "K[:3,:3] is a rotation"
  assert allclose(K[3,:3,:],0),"bottom row is 0"
  assert allclose(K[3,3,:],1),"corner is 1"
  A = Adj_UNSAFE(K)
  A.shape = (A.shape[0],A.shape[1])+sz
  return A
  
def Adj_UNSAFE(K):
  """
  The Adj operator for a rigid motion K
  
  This is the "safe" version which checks that K is a rigid motion
 
  !!!WARNING: VEC
  """
  K = asarray(K)
  if K.ndim == 2:
    K = K[...,newaxis]
  assert K.ndim == 3
  n = K.shape[2]
  t = K[:3,3,:]
  S = skew(t)
  R = K[:3,:3,:]
  z = zeros_like
  RS = array([ -dot(R[...,k],S[...,k]) for k in xrange(n) ])
  A = zeros((6,6,n),K.dtype)
  for k in xrange(n):
    Rk = R[...,k]
    A[:3,3:,k] = -dot(Rk,S[...,k])
    A[:3,:3,k] = Rk
    A[3:,3:,k] = Rk
  return A

def cayley_UNSAFE( R, inv=inv ):
  """
  Compute the Cayley transform of the matrices R
     
  C = dot(inv(R + I), (R-I))
  
  This transform takes rotations near the identity into skew matrices.
  It is its own inverse.
     
  INPUT:
    R -- N... x D x D -- a collection of orthogonal matrices
    inv -- matrix inversion function
  OUTPUT:
    N... x D x D
  """
  R = asarray(R)
  shp = R.shape
  N = int(prod(shp[:-2]))
  D = shp[-1]
  R.shape = (N,D,D)
  res = empty_like(R)
  idx = arange(D)
  for k in xrange(N):
    # Fast add, subtract identity
    A = R[k,...].copy()
    A[idx,idx] += 1
    B = R[k,...].copy()
    B[idx,idx] -= 1
    # Cayley
    res[k,...] = dot( inv(A), B )
  res.shape = shp
  R.shape = shp
  return res

def powm( M, p ):
  """
  Compute M^p for a matrix using the eigenvalue decomposition method
  
  Taking M = V diag(d) inv(V), M^p = V diag(pow(d,p)) inv(V)
  
  INPUT:
    M -- D x D -- matrix
    p -- float -- an exponent
      -- N... -- multiple exponents to be applied
  OUTPUT:
    D x D -- if p was a single float
    N... x D x D -- otherwise
  """
  M = asarray(M)
  p = asarray(p)
  assert M.shape == (M.shape[0],M.shape[0]), "Matrix is square"
  d,V = eig(M)
  res = [dot(dot(V,diag(d ** pk)), inv(V)).real for pk in p.flat]
  return asarray(res).squeeze().reshape(p.shape+M.shape)
  
def geodesicInterpolation( R0, R1, x=0.5 ):
  """
  Interpolate between R0 and R1 along the SO(D) geodesic between
  them. By default, computes the geodesic mean.
  
  INPUT:
    R0, R1 -- D x D --- orthogonal matrices
    x -- N... -- multiple interpolation positions; default is 0.5
  OUTPUT:
    D x D -- if p was a single float
    N... x D x D -- otherwise
    
  Example:
  >>> R0 = aaToRot([1,2,0])
  >>> R1 = aaToRot([1,0,2])
  >>> g = geodesicInterpolation(R0,R1)
  >>> allclose( dot(R0.T,g), dot(g.T,R1) )
  True
  """
  P = powm(dot(R0,R1.T),x)
  return dot(P,R1)

if __name__ == "__main__":
    import doctest
    doctest.testmod(optionflags=doctest.NORMALIZE_WHITESPACE)

