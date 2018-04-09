/*
 * Copyright (c) 2017-2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2008 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 *          Andreas Sandberg
 */

#include "dev/ps2/keyboard.hh"

#include "base/logging.hh"
#include "debug/PS2.hh"
#include "dev/ps2.hh"
#include "params/PS2Keyboard.hh"

const uint8_t PS2Keyboard::ID[] = {0xab, 0x83};

PS2Keyboard::PS2Keyboard(const PS2KeyboardParams *p)
    : PS2Device(p),
      lastCommand(NoCommand),
      shiftDown(false),
      enabled(false)
{
    if (p->vnc)
        p->vnc->setKeyboard(this);
}

void
PS2Keyboard::serialize(CheckpointOut &cp) const
{
    PS2Device::serialize(cp);
    SERIALIZE_SCALAR(lastCommand);
    SERIALIZE_SCALAR(shiftDown);
    SERIALIZE_SCALAR(enabled);
}

void
PS2Keyboard::unserialize(CheckpointIn &cp)
{
    PS2Device::unserialize(cp);
    UNSERIALIZE_SCALAR(lastCommand);
    UNSERIALIZE_SCALAR(shiftDown);
    UNSERIALIZE_SCALAR(enabled);
}

void
PS2Keyboard::recv(uint8_t data)
{
    if (lastCommand != NoCommand) {
        switch (lastCommand) {
          case LEDWrite:
            DPRINTF(PS2, "Setting LEDs: "
                    "caps lock %s, num lock %s, scroll lock %s\n",
                    bits(data, 2) ? "on" : "off",
                    bits(data, 1) ? "on" : "off",
                    bits(data, 0) ? "on" : "off");
            sendAck();
            lastCommand = NoCommand;
            break;
          case TypematicInfo:
            DPRINTF(PS2, "Setting typematic info to %#02x.\n", data);
            sendAck();
            lastCommand = NoCommand;
            break;
        }
        return;
    }

    switch (data) {
      case LEDWrite:
        DPRINTF(PS2, "Got LED write command.\n");
        sendAck();
        lastCommand = LEDWrite;
        break;
      case DiagnosticEcho:
        panic("Keyboard diagnostic echo unimplemented.\n");
      case AlternateScanCodes:
        panic("Accessing alternate scan codes unimplemented.\n");
      case ReadID:
        DPRINTF(PS2, "Got keyboard read ID command.\n");
        sendAck();
        send((uint8_t *)&ID, sizeof(ID));
        break;
      case TypematicInfo:
        DPRINTF(PS2, "Setting typematic info.\n");
        sendAck();
        lastCommand = TypematicInfo;
        break;
      case Enable:
        DPRINTF(PS2, "Enabling the keyboard.\n");
        enabled = true;
        sendAck();
        break;
      case Disable:
        DPRINTF(PS2, "Disabling the keyboard.\n");
        enabled = false;
        sendAck();
        break;
      case DefaultsAndDisable:
        DPRINTF(PS2, "Disabling and resetting the keyboard.\n");
        enabled = false;
        sendAck();
        break;
      case AllKeysToTypematic:
        panic("Setting all keys to typemantic unimplemented.\n");
      case AllKeysToMakeRelease:
        panic("Setting all keys to make/release unimplemented.\n");
      case AllKeysToMake:
        panic("Setting all keys to make unimplemented.\n");
      case AllKeysToTypematicMakeRelease:
        panic("Setting all keys to "
                "typematic/make/release unimplemented.\n");
      case KeyToTypematic:
        panic("Setting a key to typematic unimplemented.\n");
      case KeyToMakeRelease:
        panic("Setting a key to make/release unimplemented.\n");
      case KeyToMakeOnly:
        panic("Setting key to make only unimplemented.\n");
      case Resend:
        panic("Keyboard resend unimplemented.\n");
      case Reset:
        panic("Keyboard reset unimplemented.\n");
      default:
        panic("Unknown keyboard command %#02x.\n", data);
    }
}

void
PS2Keyboard::keyPress(uint32_t key, bool down)
{
    std::list<uint8_t> keys;

    // convert the X11 keysym into ps2 codes and update the shift
    // state (shiftDown)
    Ps2::keySymToPs2(key, down, shiftDown, keys);

    // Drop key presses if the keyboard hasn't been enabled by the
    // host. We do that after translating the key code to ensure that
    // we keep track of the shift state.
    if (!enabled)
        return;

    // Insert into our queue of characters
    for (uint8_t c : keys)
        send(c);
}


PS2Keyboard *
PS2KeyboardParams::create()
{
    return new PS2Keyboard(this);
}
