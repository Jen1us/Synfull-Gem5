/*
 * message.cpp
 *
 *  Created on: Dec 13, 2009
 *      Author: sam
 */

#include "Messages.hh"
#include <cassert>
#include <cstdlib>

using namespace std;

void StreamMessage::destroy(StreamMessage* msg)
{
    assert (msg != NULL);
    free(msg);
}