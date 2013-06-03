// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: head_s2csp_ctrl.proto

#ifndef PROTOBUF_head_5fs2csp_5fctrl_2eproto__INCLUDED
#define PROTOBUF_head_5fs2csp_5fctrl_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2004001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_message_reflection.h>
#include "component_base.pb.h"
// @@protoc_insertion_point(includes)

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_head_5fs2csp_5fctrl_2eproto();
void protobuf_AssignDesc_head_5fs2csp_5fctrl_2eproto();
void protobuf_ShutdownFile_head_5fs2csp_5fctrl_2eproto();

class M3HeadS2CSPCtrlStatus;
class M3HeadS2CSPCtrlCommand;
class M3HeadS2CSPCtrlParam;

// ===================================================================

class M3HeadS2CSPCtrlStatus : public ::google::protobuf::Message {
 public:
  M3HeadS2CSPCtrlStatus();
  virtual ~M3HeadS2CSPCtrlStatus();
  
  M3HeadS2CSPCtrlStatus(const M3HeadS2CSPCtrlStatus& from);
  
  inline M3HeadS2CSPCtrlStatus& operator=(const M3HeadS2CSPCtrlStatus& from) {
    CopyFrom(from);
    return *this;
  }
  
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }
  
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }
  
  static const ::google::protobuf::Descriptor* descriptor();
  static const M3HeadS2CSPCtrlStatus& default_instance();
  
  void Swap(M3HeadS2CSPCtrlStatus* other);
  
  // implements Message ----------------------------------------------
  
  M3HeadS2CSPCtrlStatus* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3HeadS2CSPCtrlStatus& from);
  void MergeFrom(const M3HeadS2CSPCtrlStatus& from);
  void Clear();
  bool IsInitialized() const;
  
  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  // accessors -------------------------------------------------------
  
  // optional .M3BaseStatus base = 1;
  inline bool has_base() const;
  inline void clear_base();
  static const int kBaseFieldNumber = 1;
  inline const ::M3BaseStatus& base() const;
  inline ::M3BaseStatus* mutable_base();
  inline ::M3BaseStatus* release_base();
  
  // repeated double xe = 2;
  inline int xe_size() const;
  inline void clear_xe();
  static const int kXeFieldNumber = 2;
  inline double xe(int index) const;
  inline void set_xe(int index, double value);
  inline void add_xe(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      xe() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_xe();
  
  // repeated double theta_des = 3;
  inline int theta_des_size() const;
  inline void clear_theta_des();
  static const int kThetaDesFieldNumber = 3;
  inline double theta_des(int index) const;
  inline void set_theta_des(int index, double value);
  inline void add_theta_des(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      theta_des() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_theta_des();
  
  // @@protoc_insertion_point(class_scope:M3HeadS2CSPCtrlStatus)
 private:
  inline void set_has_base();
  inline void clear_has_base();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  ::M3BaseStatus* base_;
  ::google::protobuf::RepeatedField< double > xe_;
  ::google::protobuf::RepeatedField< double > theta_des_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];
  
  friend void  protobuf_AddDesc_head_5fs2csp_5fctrl_2eproto();
  friend void protobuf_AssignDesc_head_5fs2csp_5fctrl_2eproto();
  friend void protobuf_ShutdownFile_head_5fs2csp_5fctrl_2eproto();
  
  void InitAsDefaultInstance();
  static M3HeadS2CSPCtrlStatus* default_instance_;
};
// -------------------------------------------------------------------

class M3HeadS2CSPCtrlCommand : public ::google::protobuf::Message {
 public:
  M3HeadS2CSPCtrlCommand();
  virtual ~M3HeadS2CSPCtrlCommand();
  
  M3HeadS2CSPCtrlCommand(const M3HeadS2CSPCtrlCommand& from);
  
  inline M3HeadS2CSPCtrlCommand& operator=(const M3HeadS2CSPCtrlCommand& from) {
    CopyFrom(from);
    return *this;
  }
  
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }
  
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }
  
  static const ::google::protobuf::Descriptor* descriptor();
  static const M3HeadS2CSPCtrlCommand& default_instance();
  
  void Swap(M3HeadS2CSPCtrlCommand* other);
  
  // implements Message ----------------------------------------------
  
  M3HeadS2CSPCtrlCommand* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3HeadS2CSPCtrlCommand& from);
  void MergeFrom(const M3HeadS2CSPCtrlCommand& from);
  void Clear();
  bool IsInitialized() const;
  
  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  // accessors -------------------------------------------------------
  
  // repeated double target = 1;
  inline int target_size() const;
  inline void clear_target();
  static const int kTargetFieldNumber = 1;
  inline double target(int index) const;
  inline void set_target(int index, double value);
  inline void add_target(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      target() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_target();
  
  // optional int32 enable = 2;
  inline bool has_enable() const;
  inline void clear_enable();
  static const int kEnableFieldNumber = 2;
  inline ::google::protobuf::int32 enable() const;
  inline void set_enable(::google::protobuf::int32 value);
  
  // optional double theta_des_j2 = 3;
  inline bool has_theta_des_j2() const;
  inline void clear_theta_des_j2();
  static const int kThetaDesJ2FieldNumber = 3;
  inline double theta_des_j2() const;
  inline void set_theta_des_j2(double value);
  
  // @@protoc_insertion_point(class_scope:M3HeadS2CSPCtrlCommand)
 private:
  inline void set_has_enable();
  inline void clear_has_enable();
  inline void set_has_theta_des_j2();
  inline void clear_has_theta_des_j2();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  ::google::protobuf::RepeatedField< double > target_;
  double theta_des_j2_;
  ::google::protobuf::int32 enable_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];
  
  friend void  protobuf_AddDesc_head_5fs2csp_5fctrl_2eproto();
  friend void protobuf_AssignDesc_head_5fs2csp_5fctrl_2eproto();
  friend void protobuf_ShutdownFile_head_5fs2csp_5fctrl_2eproto();
  
  void InitAsDefaultInstance();
  static M3HeadS2CSPCtrlCommand* default_instance_;
};
// -------------------------------------------------------------------

class M3HeadS2CSPCtrlParam : public ::google::protobuf::Message {
 public:
  M3HeadS2CSPCtrlParam();
  virtual ~M3HeadS2CSPCtrlParam();
  
  M3HeadS2CSPCtrlParam(const M3HeadS2CSPCtrlParam& from);
  
  inline M3HeadS2CSPCtrlParam& operator=(const M3HeadS2CSPCtrlParam& from) {
    CopyFrom(from);
    return *this;
  }
  
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }
  
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }
  
  static const ::google::protobuf::Descriptor* descriptor();
  static const M3HeadS2CSPCtrlParam& default_instance();
  
  void Swap(M3HeadS2CSPCtrlParam* other);
  
  // implements Message ----------------------------------------------
  
  M3HeadS2CSPCtrlParam* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3HeadS2CSPCtrlParam& from);
  void MergeFrom(const M3HeadS2CSPCtrlParam& from);
  void Clear();
  bool IsInitialized() const;
  
  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  // accessors -------------------------------------------------------
  
  // optional double target_slew = 1;
  inline bool has_target_slew() const;
  inline void clear_target_slew();
  static const int kTargetSlewFieldNumber = 1;
  inline double target_slew() const;
  inline void set_target_slew(double value);
  
  // repeated double slew_des = 2;
  inline int slew_des_size() const;
  inline void clear_slew_des();
  static const int kSlewDesFieldNumber = 2;
  inline double slew_des(int index) const;
  inline void set_slew_des(int index, double value);
  inline void add_slew_des(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      slew_des() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_slew_des();
  
  // repeated double theta_db = 3;
  inline int theta_db_size() const;
  inline void clear_theta_db();
  static const int kThetaDbFieldNumber = 3;
  inline double theta_db(int index) const;
  inline void set_theta_db(int index, double value);
  inline void add_theta_db(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      theta_db() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_theta_db();
  
  // optional double ratio_j0 = 4;
  inline bool has_ratio_j0() const;
  inline void clear_ratio_j0();
  static const int kRatioJ0FieldNumber = 4;
  inline double ratio_j0() const;
  inline void set_ratio_j0(double value);
  
  // repeated double origin = 5;
  inline int origin_size() const;
  inline void clear_origin();
  static const int kOriginFieldNumber = 5;
  inline double origin(int index) const;
  inline void set_origin(int index, double value);
  inline void add_origin(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      origin() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_origin();
  
  // @@protoc_insertion_point(class_scope:M3HeadS2CSPCtrlParam)
 private:
  inline void set_has_target_slew();
  inline void clear_has_target_slew();
  inline void set_has_ratio_j0();
  inline void clear_has_ratio_j0();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  double target_slew_;
  ::google::protobuf::RepeatedField< double > slew_des_;
  ::google::protobuf::RepeatedField< double > theta_db_;
  double ratio_j0_;
  ::google::protobuf::RepeatedField< double > origin_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(5 + 31) / 32];
  
  friend void  protobuf_AddDesc_head_5fs2csp_5fctrl_2eproto();
  friend void protobuf_AssignDesc_head_5fs2csp_5fctrl_2eproto();
  friend void protobuf_ShutdownFile_head_5fs2csp_5fctrl_2eproto();
  
  void InitAsDefaultInstance();
  static M3HeadS2CSPCtrlParam* default_instance_;
};
// ===================================================================


// ===================================================================

// M3HeadS2CSPCtrlStatus

// optional .M3BaseStatus base = 1;
inline bool M3HeadS2CSPCtrlStatus::has_base() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3HeadS2CSPCtrlStatus::set_has_base() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3HeadS2CSPCtrlStatus::clear_has_base() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3HeadS2CSPCtrlStatus::clear_base() {
  if (base_ != NULL) base_->::M3BaseStatus::Clear();
  clear_has_base();
}
inline const ::M3BaseStatus& M3HeadS2CSPCtrlStatus::base() const {
  return base_ != NULL ? *base_ : *default_instance_->base_;
}
inline ::M3BaseStatus* M3HeadS2CSPCtrlStatus::mutable_base() {
  set_has_base();
  if (base_ == NULL) base_ = new ::M3BaseStatus;
  return base_;
}
inline ::M3BaseStatus* M3HeadS2CSPCtrlStatus::release_base() {
  clear_has_base();
  ::M3BaseStatus* temp = base_;
  base_ = NULL;
  return temp;
}

// repeated double xe = 2;
inline int M3HeadS2CSPCtrlStatus::xe_size() const {
  return xe_.size();
}
inline void M3HeadS2CSPCtrlStatus::clear_xe() {
  xe_.Clear();
}
inline double M3HeadS2CSPCtrlStatus::xe(int index) const {
  return xe_.Get(index);
}
inline void M3HeadS2CSPCtrlStatus::set_xe(int index, double value) {
  xe_.Set(index, value);
}
inline void M3HeadS2CSPCtrlStatus::add_xe(double value) {
  xe_.Add(value);
}
inline const ::google::protobuf::RepeatedField< double >&
M3HeadS2CSPCtrlStatus::xe() const {
  return xe_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HeadS2CSPCtrlStatus::mutable_xe() {
  return &xe_;
}

// repeated double theta_des = 3;
inline int M3HeadS2CSPCtrlStatus::theta_des_size() const {
  return theta_des_.size();
}
inline void M3HeadS2CSPCtrlStatus::clear_theta_des() {
  theta_des_.Clear();
}
inline double M3HeadS2CSPCtrlStatus::theta_des(int index) const {
  return theta_des_.Get(index);
}
inline void M3HeadS2CSPCtrlStatus::set_theta_des(int index, double value) {
  theta_des_.Set(index, value);
}
inline void M3HeadS2CSPCtrlStatus::add_theta_des(double value) {
  theta_des_.Add(value);
}
inline const ::google::protobuf::RepeatedField< double >&
M3HeadS2CSPCtrlStatus::theta_des() const {
  return theta_des_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HeadS2CSPCtrlStatus::mutable_theta_des() {
  return &theta_des_;
}

// -------------------------------------------------------------------

// M3HeadS2CSPCtrlCommand

// repeated double target = 1;
inline int M3HeadS2CSPCtrlCommand::target_size() const {
  return target_.size();
}
inline void M3HeadS2CSPCtrlCommand::clear_target() {
  target_.Clear();
}
inline double M3HeadS2CSPCtrlCommand::target(int index) const {
  return target_.Get(index);
}
inline void M3HeadS2CSPCtrlCommand::set_target(int index, double value) {
  target_.Set(index, value);
}
inline void M3HeadS2CSPCtrlCommand::add_target(double value) {
  target_.Add(value);
}
inline const ::google::protobuf::RepeatedField< double >&
M3HeadS2CSPCtrlCommand::target() const {
  return target_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HeadS2CSPCtrlCommand::mutable_target() {
  return &target_;
}

// optional int32 enable = 2;
inline bool M3HeadS2CSPCtrlCommand::has_enable() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void M3HeadS2CSPCtrlCommand::set_has_enable() {
  _has_bits_[0] |= 0x00000002u;
}
inline void M3HeadS2CSPCtrlCommand::clear_has_enable() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void M3HeadS2CSPCtrlCommand::clear_enable() {
  enable_ = 0;
  clear_has_enable();
}
inline ::google::protobuf::int32 M3HeadS2CSPCtrlCommand::enable() const {
  return enable_;
}
inline void M3HeadS2CSPCtrlCommand::set_enable(::google::protobuf::int32 value) {
  set_has_enable();
  enable_ = value;
}

// optional double theta_des_j2 = 3;
inline bool M3HeadS2CSPCtrlCommand::has_theta_des_j2() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void M3HeadS2CSPCtrlCommand::set_has_theta_des_j2() {
  _has_bits_[0] |= 0x00000004u;
}
inline void M3HeadS2CSPCtrlCommand::clear_has_theta_des_j2() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void M3HeadS2CSPCtrlCommand::clear_theta_des_j2() {
  theta_des_j2_ = 0;
  clear_has_theta_des_j2();
}
inline double M3HeadS2CSPCtrlCommand::theta_des_j2() const {
  return theta_des_j2_;
}
inline void M3HeadS2CSPCtrlCommand::set_theta_des_j2(double value) {
  set_has_theta_des_j2();
  theta_des_j2_ = value;
}

// -------------------------------------------------------------------

// M3HeadS2CSPCtrlParam

// optional double target_slew = 1;
inline bool M3HeadS2CSPCtrlParam::has_target_slew() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3HeadS2CSPCtrlParam::set_has_target_slew() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3HeadS2CSPCtrlParam::clear_has_target_slew() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3HeadS2CSPCtrlParam::clear_target_slew() {
  target_slew_ = 0;
  clear_has_target_slew();
}
inline double M3HeadS2CSPCtrlParam::target_slew() const {
  return target_slew_;
}
inline void M3HeadS2CSPCtrlParam::set_target_slew(double value) {
  set_has_target_slew();
  target_slew_ = value;
}

// repeated double slew_des = 2;
inline int M3HeadS2CSPCtrlParam::slew_des_size() const {
  return slew_des_.size();
}
inline void M3HeadS2CSPCtrlParam::clear_slew_des() {
  slew_des_.Clear();
}
inline double M3HeadS2CSPCtrlParam::slew_des(int index) const {
  return slew_des_.Get(index);
}
inline void M3HeadS2CSPCtrlParam::set_slew_des(int index, double value) {
  slew_des_.Set(index, value);
}
inline void M3HeadS2CSPCtrlParam::add_slew_des(double value) {
  slew_des_.Add(value);
}
inline const ::google::protobuf::RepeatedField< double >&
M3HeadS2CSPCtrlParam::slew_des() const {
  return slew_des_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HeadS2CSPCtrlParam::mutable_slew_des() {
  return &slew_des_;
}

// repeated double theta_db = 3;
inline int M3HeadS2CSPCtrlParam::theta_db_size() const {
  return theta_db_.size();
}
inline void M3HeadS2CSPCtrlParam::clear_theta_db() {
  theta_db_.Clear();
}
inline double M3HeadS2CSPCtrlParam::theta_db(int index) const {
  return theta_db_.Get(index);
}
inline void M3HeadS2CSPCtrlParam::set_theta_db(int index, double value) {
  theta_db_.Set(index, value);
}
inline void M3HeadS2CSPCtrlParam::add_theta_db(double value) {
  theta_db_.Add(value);
}
inline const ::google::protobuf::RepeatedField< double >&
M3HeadS2CSPCtrlParam::theta_db() const {
  return theta_db_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HeadS2CSPCtrlParam::mutable_theta_db() {
  return &theta_db_;
}

// optional double ratio_j0 = 4;
inline bool M3HeadS2CSPCtrlParam::has_ratio_j0() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void M3HeadS2CSPCtrlParam::set_has_ratio_j0() {
  _has_bits_[0] |= 0x00000008u;
}
inline void M3HeadS2CSPCtrlParam::clear_has_ratio_j0() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void M3HeadS2CSPCtrlParam::clear_ratio_j0() {
  ratio_j0_ = 0;
  clear_has_ratio_j0();
}
inline double M3HeadS2CSPCtrlParam::ratio_j0() const {
  return ratio_j0_;
}
inline void M3HeadS2CSPCtrlParam::set_ratio_j0(double value) {
  set_has_ratio_j0();
  ratio_j0_ = value;
}

// repeated double origin = 5;
inline int M3HeadS2CSPCtrlParam::origin_size() const {
  return origin_.size();
}
inline void M3HeadS2CSPCtrlParam::clear_origin() {
  origin_.Clear();
}
inline double M3HeadS2CSPCtrlParam::origin(int index) const {
  return origin_.Get(index);
}
inline void M3HeadS2CSPCtrlParam::set_origin(int index, double value) {
  origin_.Set(index, value);
}
inline void M3HeadS2CSPCtrlParam::add_origin(double value) {
  origin_.Add(value);
}
inline const ::google::protobuf::RepeatedField< double >&
M3HeadS2CSPCtrlParam::origin() const {
  return origin_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HeadS2CSPCtrlParam::mutable_origin() {
  return &origin_;
}


// @@protoc_insertion_point(namespace_scope)

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_head_5fs2csp_5fctrl_2eproto__INCLUDED