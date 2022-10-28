%module mip_sdk
%include "typemaps.i"
%feature("flatnested", "1");

%{
#include "mip/mip_all.h"

// Required since swig will flatten out all the namespces
using namespace mip;
using namespace mip::C;

// Some helper functions to make it easier to do C/C++ things from high level languages
uint8_t* malloc_u8(int n) {
    return (uint8_t*)malloc(n);
}

%}

//System functions required to do some memory stuff
void* malloc(int);
void free(void*);

// Expose our helper functions
uint8_t* malloc_u8(int);

// Typemaps for some primitive typedefs
%apply unsigned long { mip::C::timeout_type };
%apply unsigned long { mip::C::timestamp_type };

//MIP Core
%include "mip/mip_result.h"
%include "mip/mip_types.h"
%include "mip/mip_cmdqueue.h"
%include "mip/mip_dispatch.h"
%include "mip/mip_field.h"
%include "mip/mip_interface.h"
%include "mip/mip_offsets.h"
%include "mip/mip_packet.h"
%include "mip/mip_parser.h"
%include "mip/definitions/descriptors.h"

//MIP Utils
%include "mip/utils/serialization.h"

//MIP Commands
%include "mip/definitions/commands_base.h"
%include "mip/definitions/commands_3dm.h"
%include "mip/definitions/commands_filter.h"
%include "mip/definitions/commands_gnss.h"
%include "mip/definitions/commands_rtk.h"
%include "mip/definitions/commands_system.h"

//MIP Data
%include "mip/definitions/data_shared.h"
%include "mip/definitions/data_system.h"
%include "mip/definitions/data_sensor.h"
%include "mip/definitions/data_gnss.h"
%include "mip/definitions/data_filter.h"

//SDK version
%include "mip/mip_version.h"

// Callback implementations differ between languages

// Node callback implementation
#ifdef SWIG_JAVASCRIPT_V8
%{

#include <stdio.h>

#include <chrono>

#include <node_buffer.h>

namespace mip_js {

// Globally store the callbacks
v8::Persistent<v8::Function> recv_fn;
v8::Persistent<v8::Function> send_fn;

// Functions to store the respective callbacks
void mip_interface_set_recv_callback(const v8::FunctionCallbackInfo<v8::Value>& args) {
    if (args[0]->IsFunction()) {
        recv_fn.Reset(args.GetIsolate(), v8::Local<v8::Function>::Cast(args[0]));
    } else {
        recv_fn.Reset();
    }
}

void mip_interface_set_send_callback(const v8::FunctionCallbackInfo<v8::Value>& args) {
    if (args[0]->IsFunction()) {
        send_fn.Reset(args.GetIsolate(), v8::Local<v8::Function>::Cast(args[0]));
    } else {
        send_fn.Reset();
    }
}

}  // namespace mip_js

// Expose the required functions for the MIP SDK
extern "C" {
bool mip_interface_user_recv_from_device(mip_interface* device, uint8_t* buffer, size_t max_length, size_t* out_length, timestamp_type* timestamp_out) {
    // Get a local copy of the node callback
    v8::Isolate* isolate = v8::Isolate::GetCurrent();
    v8::Local<v8::Function> recv_fn_local = v8::Local<v8::Function>::New(isolate, mip_js::recv_fn);
    if (!recv_fn_local.IsEmpty()) {
        // Build parameters and call the JS function
        const uint32_t argc = 1;
        v8::Local<v8::Value> argv[argc] = {
            v8::BigInt::New(isolate, max_length)
        };
        v8::MaybeLocal<v8::Value> result = recv_fn_local->Call(v8::Null(isolate), argc, argv);

        // Assume that the return type was a buffer
        if (result.IsEmpty() || !node::Buffer::HasInstance(result.ToLocalChecked()))
            return false;

        // Extract the data from the buffer
        const char* node_buffer_data = node::Buffer::Data(result.ToLocalChecked());
        const size_t node_buffer_size = node::Buffer::Length(result.ToLocalChecked());

        // Copy the data back into the buffer
        *out_length = node_buffer_size < max_length ? node_buffer_size : max_length;
        memcpy(buffer, node_buffer_data, *out_length);

        // Fill out the timestamp
        {
            using namespace std::chrono;
            *timestamp_out = duration_cast<milliseconds>( steady_clock::now().time_since_epoch() ).count();
        }

        return true;
    }
    return false;
}


bool mip_interface_user_send_to_device(mip_interface* device, const uint8_t* data, size_t length) {
    // Get a local copy of the node callback
    v8::Isolate* isolate = v8::Isolate::GetCurrent();
    v8::Local<v8::Function> send_fn_local = v8::Local<v8::Function>::New(isolate, mip_js::send_fn);
    if (!send_fn_local.IsEmpty()) {
        // Make a node buffer from the data
        v8::MaybeLocal<v8::Object> buffer = node::Buffer::Copy(isolate, (char*)data, length);

        // Build parameters and call the JS function
        const uint32_t argc = 1;
        v8::Local<v8::Value> argv[argc] = {
            buffer.ToLocalChecked()
        };
        v8::MaybeLocal<v8::Value> result = send_fn_local->Call(v8::Null(isolate), argc, argv);

        // If we didn't get a return type, or it was not a boolean, return true
        if (result.IsEmpty() || !result.ToLocalChecked()->IsBoolean())
            return true;

        // If the return type was a boolean, return it
        return v8::Local<v8::Boolean>::Cast(result.ToLocalChecked())->Value();
    }
    return false;
}
}
%}

// Expose the functions to node. We need to expose these explicitly, otherwise swig will mess them up
%init %{
SWIGV8_AddStaticFunction(exports_obj, "mip_interface_set_recv_callback", mip_js::mip_interface_set_recv_callback);
SWIGV8_AddStaticFunction(exports_obj, "mip_interface_set_send_callback", mip_js::mip_interface_set_send_callback);
%}

#endif