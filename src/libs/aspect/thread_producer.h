
/***************************************************************************
 *  thread_producer.h - Thread producer aspect for Fawkes
 *
 *  Created: Tue Nov 20 11:24:35 2007
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __ASPECT_THREAD_PRODUCER_H_
#define __ASPECT_THREAD_PRODUCER_H_

#include <aspect/aspect.h>
#include <core/threading/thread_collector.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ThreadProducerAspect : public virtual Aspect
{
 public:
  ThreadProducerAspect();
  virtual ~ThreadProducerAspect();

  void init_ThreadProducerAspect(ThreadCollector *collector);

 protected:
  ThreadCollector *thread_collector;
};

} // end namespace fawkes

#endif
