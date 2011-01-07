/**
 * You received this file as part of an advanced experimental
 * robotics framework prototype ('finroc')
 *
 * Copyright (C) 2007-2010 Max Reichardt,
 *   Robotics Research Lab, University of Kaiserslautern
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#include "plugins/blackboard/tBlackboardBuffer.h"
#include "plugins/blackboard/tBlackboardPlugin.h"
#include "core/buffers/tCoreInput.h"
#include "rrlib/finroc_core_utils/stream/tMemoryBuffer.h"
#include "rrlib/finroc_core_utils/stream/tFixedBuffer.h"
#include "core/buffers/tCoreOutput.h"

namespace finroc
{
namespace blackboard
{
core::tDataType* tBlackboardBuffer::cBUFFER_TYPE = tBlackboardPlugin::RegisterBlackboardType(util::tTypedClass<tBlackboardBuffer>(), "Raw Blackboard Data");

void tBlackboardBuffer::Deserialize(core::tCoreInput& is)
{
  lock_iD = is.ReadInt();
  bb_capacity = is.ReadInt();
  elements = is.ReadInt();
  element_size = is.ReadInt();
  ::finroc::core::tMemBuffer::Deserialize(is);
}

void tBlackboardBuffer::Resize(int new_capacity, int new_elements, int new_element_size, bool keep_contents)
{
  // remember old values
  int old_elem_size = element_size;
  //int oldCapacity = bbCapacity;
  int old_elements = elements;
  int old_end = old_elem_size * old_elements;

  // set new values
  bb_capacity = new_capacity;
  elements = new_elements;
  element_size = new_element_size < 0 ? element_size : new_element_size;
  int new_end = element_size * bb_capacity;

  EnsureCapacity(new_end, keep_contents, old_end);
  if (!keep_contents || element_size == old_elem_size)
  {
    this->cur_size = new_elements * new_element_size;
    if (old_end < new_end)    // zero new memory out
    {
      this->backend->ZeroOut(old_end, new_end - old_end);
    }
    return;
  }

  // do some copying
  int copy_elems = std::min(old_elements, elements);
  int copy_size = std::min(old_elem_size, element_size);

  if (element_size < old_elem_size)
  {
    for (int i = 1; i < copy_elems; i++)
    {
      this->backend->Put(i * element_size, *this->backend, i * old_elem_size, copy_size);
    }
  }
  else
  {
    // new element size > old element size... we need to be careful in this case => copy from back to front
    for (int i = copy_elems - 1; i > 0; i--)
    {
      // copy from back to front
      int src_ptr = i * old_elem_size + copy_size;
      int dest_ptr = i * element_size + copy_size;
      this->backend->ZeroOut(dest_ptr, element_size - old_elem_size);
      for (int j = 0; j < copy_size; j++)
      {
        src_ptr--;
        dest_ptr--;
        this->backend->PutByte(dest_ptr, this->backend->GetByte(src_ptr));
      }
    }
  }

  if (old_end < new_end)    // zero new memory out
  {
    this->backend->ZeroOut(old_end, new_end - old_end);
  }

  this->cur_size = new_elements * new_element_size;
}

void tBlackboardBuffer::Serialize(core::tCoreOutput& os) const
{
  os.WriteInt(lock_iD);
  os.WriteInt(bb_capacity);
  os.WriteInt(elements);
  os.WriteInt(element_size);
  ::finroc::core::tMemBuffer::Serialize(os);
}

} // namespace finroc
} // namespace blackboard

