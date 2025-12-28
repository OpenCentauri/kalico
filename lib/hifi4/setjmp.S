/* setjmp/longjmp functions for Xtensa.

   Copyright (c) 2001-2006 by Tensilica Inc.

   Permission is hereby granted, free of charge, to any person obtaining
   a copy of this software and associated documentation files (the
   "Software"), to deal in the Software without restriction, including
   without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to
   permit persons to whom the Software is furnished to do so, subject to
   the following conditions:

   The above copyright notice and this permission notice shall be included
   in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
   CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.  */

	.text
	.align	4
	.literal_position
	.global	setjmp
	.type	setjmp, @function
setjmp:
	s32i	a0, a2, 0
	s32i	a1, a2, 4
	s32i	a12, a2, 8
	s32i	a13, a2, 12
	s32i	a14, a2, 16
	s32i	a15, a2, 20
	movi	a2, 0
	ret
	.size	setjmp, . - setjmp

	.align	4
	.literal_position
	.global	longjmp
	.type	longjmp, @function
longjmp:
	l32i	a0, a2, 0
	l32i	a12, a2, 8
	l32i	a13, a2, 12
	l32i	a14, a2, 16
	l32i	a15, a2, 20
	l32i	a1, a2, 4
	/* Return val ? val : 1.  */
	movi	a2, 1
	movnez	a2, a3, a3

	ret
	.size	longjmp, .-longjmp
