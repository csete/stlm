/* -*- c++ -*- */
/*
 * Copyright 2013 Alexandru Csete, OZ9AEC
 *
 * Strx is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Strx is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Gqrx; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_STRX_API_H
#define INCLUDED_STRX_API_H

#include <gnuradio/attributes.h>

#ifdef gnuradio_strx_EXPORTS
#  define STRX_API __GR_ATTR_EXPORT
#else
#  define STRX_API __GR_ATTR_IMPORT
#endif

#endif /* INCLUDED_STRX_API_H */
