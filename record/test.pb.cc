// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: test.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "test.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
// @@protoc_insertion_point(includes)

namespace pb {

void protobuf_ShutdownFile_test_2eproto() {
  delete serial_msg::default_instance_;
  delete msg_bag::default_instance_;
}

#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
void protobuf_AddDesc_test_2eproto_impl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#else
void protobuf_AddDesc_test_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_test_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#endif
  serial_msg::default_instance_ = new serial_msg();
  msg_bag::default_instance_ = new msg_bag();
  serial_msg::default_instance_->InitAsDefaultInstance();
  msg_bag::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_test_2eproto);
}

#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AddDesc_test_2eproto_once_);
void protobuf_AddDesc_test_2eproto() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AddDesc_test_2eproto_once_,
                 &protobuf_AddDesc_test_2eproto_impl);
}
#else
// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_test_2eproto {
  StaticDescriptorInitializer_test_2eproto() {
    protobuf_AddDesc_test_2eproto();
  }
} static_descriptor_initializer_test_2eproto_;
#endif

// ===================================================================

bool serial_msg_Msg_type_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const serial_msg_Msg_type serial_msg::GPS;
const serial_msg_Msg_type serial_msg::POSE;
const serial_msg_Msg_type serial_msg::VIEWANGLE;
const serial_msg_Msg_type serial_msg::OTHER;
const serial_msg_Msg_type serial_msg::Msg_type_MIN;
const serial_msg_Msg_type serial_msg::Msg_type_MAX;
const int serial_msg::Msg_type_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int serial_msg::kTimestampFieldNumber;
const int serial_msg::kTypeFieldNumber;
const int serial_msg::kLenFieldNumber;
const int serial_msg::kBodyFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

serial_msg::serial_msg()
  : ::google::protobuf::MessageLite(), _arena_ptr_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.serial_msg)
}

void serial_msg::InitAsDefaultInstance() {
  _is_default_instance_ = true;
}

serial_msg::serial_msg(const serial_msg& from)
  : ::google::protobuf::MessageLite(),
    _arena_ptr_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:pb.serial_msg)
}

void serial_msg::SharedCtor() {
    _is_default_instance_ = false;
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  timestamp_ = GOOGLE_ULONGLONG(0);
  type_ = 0;
  len_ = 0u;
  body_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

serial_msg::~serial_msg() {
  // @@protoc_insertion_point(destructor:pb.serial_msg)
  SharedDtor();
}

void serial_msg::SharedDtor() {
  body_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  if (this != &default_instance()) {
  #else
  if (this != default_instance_) {
  #endif
  }
}

void serial_msg::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const serial_msg& serial_msg::default_instance() {
#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  protobuf_AddDesc_test_2eproto();
#else
  if (default_instance_ == NULL) protobuf_AddDesc_test_2eproto();
#endif
  return *default_instance_;
}

serial_msg* serial_msg::default_instance_ = NULL;

serial_msg* serial_msg::New(::google::protobuf::Arena* arena) const {
  serial_msg* n = new serial_msg;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void serial_msg::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.serial_msg)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(serial_msg, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<serial_msg*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  ZR_(timestamp_, len_);
  body_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());

#undef ZR_HELPER_
#undef ZR_

}

bool serial_msg::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:pb.serial_msg)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional uint64 timestamp = 1;
      case 1: {
        if (tag == 8) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 input, &timestamp_)));

        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_type;
        break;
      }

      // optional .pb.serial_msg.Msg_type type = 2;
      case 2: {
        if (tag == 16) {
         parse_type:
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          set_type(static_cast< ::pb::serial_msg_Msg_type >(value));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(24)) goto parse_len;
        break;
      }

      // optional uint32 len = 3;
      case 3: {
        if (tag == 24) {
         parse_len:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &len_)));

        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(34)) goto parse_body;
        break;
      }

      // optional bytes body = 4;
      case 4: {
        if (tag == 34) {
         parse_body:
          DO_(::google::protobuf::internal::WireFormatLite::ReadBytes(
                input, this->mutable_body()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:pb.serial_msg)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:pb.serial_msg)
  return false;
#undef DO_
}

void serial_msg::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:pb.serial_msg)
  // optional uint64 timestamp = 1;
  if (this->timestamp() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt64(1, this->timestamp(), output);
  }

  // optional .pb.serial_msg.Msg_type type = 2;
  if (this->type() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      2, this->type(), output);
  }

  // optional uint32 len = 3;
  if (this->len() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(3, this->len(), output);
  }

  // optional bytes body = 4;
  if (this->body().size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBytesMaybeAliased(
      4, this->body(), output);
  }

  // @@protoc_insertion_point(serialize_end:pb.serial_msg)
}

int serial_msg::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:pb.serial_msg)
  int total_size = 0;

  // optional uint64 timestamp = 1;
  if (this->timestamp() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt64Size(
        this->timestamp());
  }

  // optional .pb.serial_msg.Msg_type type = 2;
  if (this->type() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::EnumSize(this->type());
  }

  // optional uint32 len = 3;
  if (this->len() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->len());
  }

  // optional bytes body = 4;
  if (this->body().size() > 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::BytesSize(
        this->body());
  }

  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void serial_msg::CheckTypeAndMergeFrom(
    const ::google::protobuf::MessageLite& from) {
  MergeFrom(*::google::protobuf::down_cast<const serial_msg*>(&from));
}

void serial_msg::MergeFrom(const serial_msg& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.serial_msg)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from.timestamp() != 0) {
    set_timestamp(from.timestamp());
  }
  if (from.type() != 0) {
    set_type(from.type());
  }
  if (from.len() != 0) {
    set_len(from.len());
  }
  if (from.body().size() > 0) {

    body_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.body_);
  }
}

void serial_msg::CopyFrom(const serial_msg& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.serial_msg)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool serial_msg::IsInitialized() const {

  return true;
}

void serial_msg::Swap(serial_msg* other) {
  if (other == this) return;
  InternalSwap(other);
}
void serial_msg::InternalSwap(serial_msg* other) {
  std::swap(timestamp_, other->timestamp_);
  std::swap(type_, other->type_);
  std::swap(len_, other->len_);
  body_.Swap(&other->body_);
  _unknown_fields_.Swap(&other->_unknown_fields_);
  std::swap(_cached_size_, other->_cached_size_);
}

::std::string serial_msg::GetTypeName() const {
  return "pb.serial_msg";
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// serial_msg

// optional uint64 timestamp = 1;
void serial_msg::clear_timestamp() {
  timestamp_ = GOOGLE_ULONGLONG(0);
}
 ::google::protobuf::uint64 serial_msg::timestamp() const {
  // @@protoc_insertion_point(field_get:pb.serial_msg.timestamp)
  return timestamp_;
}
 void serial_msg::set_timestamp(::google::protobuf::uint64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:pb.serial_msg.timestamp)
}

// optional .pb.serial_msg.Msg_type type = 2;
void serial_msg::clear_type() {
  type_ = 0;
}
 ::pb::serial_msg_Msg_type serial_msg::type() const {
  // @@protoc_insertion_point(field_get:pb.serial_msg.type)
  return static_cast< ::pb::serial_msg_Msg_type >(type_);
}
 void serial_msg::set_type(::pb::serial_msg_Msg_type value) {
  
  type_ = value;
  // @@protoc_insertion_point(field_set:pb.serial_msg.type)
}

// optional uint32 len = 3;
void serial_msg::clear_len() {
  len_ = 0u;
}
 ::google::protobuf::uint32 serial_msg::len() const {
  // @@protoc_insertion_point(field_get:pb.serial_msg.len)
  return len_;
}
 void serial_msg::set_len(::google::protobuf::uint32 value) {
  
  len_ = value;
  // @@protoc_insertion_point(field_set:pb.serial_msg.len)
}

// optional bytes body = 4;
void serial_msg::clear_body() {
  body_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 const ::std::string& serial_msg::body() const {
  // @@protoc_insertion_point(field_get:pb.serial_msg.body)
  return body_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void serial_msg::set_body(const ::std::string& value) {
  
  body_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:pb.serial_msg.body)
}
 void serial_msg::set_body(const char* value) {
  
  body_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:pb.serial_msg.body)
}
 void serial_msg::set_body(const void* value, size_t size) {
  
  body_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:pb.serial_msg.body)
}
 ::std::string* serial_msg::mutable_body() {
  
  // @@protoc_insertion_point(field_mutable:pb.serial_msg.body)
  return body_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 ::std::string* serial_msg::release_body() {
  // @@protoc_insertion_point(field_release:pb.serial_msg.body)
  
  return body_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
 void serial_msg::set_allocated_body(::std::string* body) {
  if (body != NULL) {
    
  } else {
    
  }
  body_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), body);
  // @@protoc_insertion_point(field_set_allocated:pb.serial_msg.body)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int msg_bag::kMsgsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

msg_bag::msg_bag()
  : ::google::protobuf::MessageLite(), _arena_ptr_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:pb.msg_bag)
}

void msg_bag::InitAsDefaultInstance() {
  _is_default_instance_ = true;
}

msg_bag::msg_bag(const msg_bag& from)
  : ::google::protobuf::MessageLite(),
    _arena_ptr_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:pb.msg_bag)
}

void msg_bag::SharedCtor() {
    _is_default_instance_ = false;
  _cached_size_ = 0;
}

msg_bag::~msg_bag() {
  // @@protoc_insertion_point(destructor:pb.msg_bag)
  SharedDtor();
}

void msg_bag::SharedDtor() {
  #ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  if (this != &default_instance()) {
  #else
  if (this != default_instance_) {
  #endif
  }
}

void msg_bag::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const msg_bag& msg_bag::default_instance() {
#ifdef GOOGLE_PROTOBUF_NO_STATIC_INITIALIZER
  protobuf_AddDesc_test_2eproto();
#else
  if (default_instance_ == NULL) protobuf_AddDesc_test_2eproto();
#endif
  return *default_instance_;
}

msg_bag* msg_bag::default_instance_ = NULL;

msg_bag* msg_bag::New(::google::protobuf::Arena* arena) const {
  msg_bag* n = new msg_bag;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void msg_bag::Clear() {
// @@protoc_insertion_point(message_clear_start:pb.msg_bag)
  msgs_.Clear();
}

bool msg_bag::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:pb.msg_bag)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .pb.serial_msg msgs = 1;
      case 1: {
        if (tag == 10) {
          DO_(input->IncrementRecursionDepth());
         parse_loop_msgs:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtualNoRecursionDepth(
                input, add_msgs()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(10)) goto parse_loop_msgs;
        input->UnsafeDecrementRecursionDepth();
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:pb.msg_bag)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:pb.msg_bag)
  return false;
#undef DO_
}

void msg_bag::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:pb.msg_bag)
  // repeated .pb.serial_msg msgs = 1;
  for (unsigned int i = 0, n = this->msgs_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessage(
      1, this->msgs(i), output);
  }

  // @@protoc_insertion_point(serialize_end:pb.msg_bag)
}

int msg_bag::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:pb.msg_bag)
  int total_size = 0;

  // repeated .pb.serial_msg msgs = 1;
  total_size += 1 * this->msgs_size();
  for (int i = 0; i < this->msgs_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->msgs(i));
  }

  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void msg_bag::CheckTypeAndMergeFrom(
    const ::google::protobuf::MessageLite& from) {
  MergeFrom(*::google::protobuf::down_cast<const msg_bag*>(&from));
}

void msg_bag::MergeFrom(const msg_bag& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:pb.msg_bag)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  msgs_.MergeFrom(from.msgs_);
}

void msg_bag::CopyFrom(const msg_bag& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:pb.msg_bag)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool msg_bag::IsInitialized() const {

  return true;
}

void msg_bag::Swap(msg_bag* other) {
  if (other == this) return;
  InternalSwap(other);
}
void msg_bag::InternalSwap(msg_bag* other) {
  msgs_.UnsafeArenaSwap(&other->msgs_);
  _unknown_fields_.Swap(&other->_unknown_fields_);
  std::swap(_cached_size_, other->_cached_size_);
}

::std::string msg_bag::GetTypeName() const {
  return "pb.msg_bag";
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// msg_bag

// repeated .pb.serial_msg msgs = 1;
int msg_bag::msgs_size() const {
  return msgs_.size();
}
void msg_bag::clear_msgs() {
  msgs_.Clear();
}
const ::pb::serial_msg& msg_bag::msgs(int index) const {
  // @@protoc_insertion_point(field_get:pb.msg_bag.msgs)
  return msgs_.Get(index);
}
::pb::serial_msg* msg_bag::mutable_msgs(int index) {
  // @@protoc_insertion_point(field_mutable:pb.msg_bag.msgs)
  return msgs_.Mutable(index);
}
::pb::serial_msg* msg_bag::add_msgs() {
  // @@protoc_insertion_point(field_add:pb.msg_bag.msgs)
  return msgs_.Add();
}
::google::protobuf::RepeatedPtrField< ::pb::serial_msg >*
msg_bag::mutable_msgs() {
  // @@protoc_insertion_point(field_mutable_list:pb.msg_bag.msgs)
  return &msgs_;
}
const ::google::protobuf::RepeatedPtrField< ::pb::serial_msg >&
msg_bag::msgs() const {
  // @@protoc_insertion_point(field_list:pb.msg_bag.msgs)
  return msgs_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace pb

// @@protoc_insertion_point(global_scope)
