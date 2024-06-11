// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: offline_bbox.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_offline_5fbbox_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_offline_5fbbox_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3020000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3020000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_offline_5fbbox_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_offline_5fbbox_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_offline_5fbbox_2eproto;
namespace offline_bbox {
class Bbox;
struct BboxDefaultTypeInternal;
extern BboxDefaultTypeInternal _Bbox_default_instance_;
class offline_bboxes;
struct offline_bboxesDefaultTypeInternal;
extern offline_bboxesDefaultTypeInternal _offline_bboxes_default_instance_;
}  // namespace offline_bbox
PROTOBUF_NAMESPACE_OPEN
template<> ::offline_bbox::Bbox* Arena::CreateMaybeMessage<::offline_bbox::Bbox>(Arena*);
template<> ::offline_bbox::offline_bboxes* Arena::CreateMaybeMessage<::offline_bbox::offline_bboxes>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace offline_bbox {

// ===================================================================

class Bbox final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:offline_bbox.Bbox) */ {
 public:
  inline Bbox() : Bbox(nullptr) {}
  ~Bbox() override;
  explicit PROTOBUF_CONSTEXPR Bbox(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Bbox(const Bbox& from);
  Bbox(Bbox&& from) noexcept
    : Bbox() {
    *this = ::std::move(from);
  }

  inline Bbox& operator=(const Bbox& from) {
    CopyFrom(from);
    return *this;
  }
  inline Bbox& operator=(Bbox&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Bbox& default_instance() {
    return *internal_default_instance();
  }
  static inline const Bbox* internal_default_instance() {
    return reinterpret_cast<const Bbox*>(
               &_Bbox_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Bbox& a, Bbox& b) {
    a.Swap(&b);
  }
  inline void Swap(Bbox* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Bbox* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Bbox* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Bbox>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Bbox& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Bbox& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Bbox* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "offline_bbox.Bbox";
  }
  protected:
  explicit Bbox(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kObjIdFieldNumber = 1,
    kBboxXFieldNumber = 2,
    kBboxYFieldNumber = 3,
    kBboxWFieldNumber = 4,
    kBboxHFieldNumber = 5,
    kBboxPFieldNumber = 6,
  };
  // optional int32 obj_id = 1;
  bool has_obj_id() const;
  private:
  bool _internal_has_obj_id() const;
  public:
  void clear_obj_id();
  int32_t obj_id() const;
  void set_obj_id(int32_t value);
  private:
  int32_t _internal_obj_id() const;
  void _internal_set_obj_id(int32_t value);
  public:

  // optional int32 bbox_x = 2;
  bool has_bbox_x() const;
  private:
  bool _internal_has_bbox_x() const;
  public:
  void clear_bbox_x();
  int32_t bbox_x() const;
  void set_bbox_x(int32_t value);
  private:
  int32_t _internal_bbox_x() const;
  void _internal_set_bbox_x(int32_t value);
  public:

  // optional int32 bbox_y = 3;
  bool has_bbox_y() const;
  private:
  bool _internal_has_bbox_y() const;
  public:
  void clear_bbox_y();
  int32_t bbox_y() const;
  void set_bbox_y(int32_t value);
  private:
  int32_t _internal_bbox_y() const;
  void _internal_set_bbox_y(int32_t value);
  public:

  // optional int32 bbox_w = 4;
  bool has_bbox_w() const;
  private:
  bool _internal_has_bbox_w() const;
  public:
  void clear_bbox_w();
  int32_t bbox_w() const;
  void set_bbox_w(int32_t value);
  private:
  int32_t _internal_bbox_w() const;
  void _internal_set_bbox_w(int32_t value);
  public:

  // optional int32 bbox_h = 5;
  bool has_bbox_h() const;
  private:
  bool _internal_has_bbox_h() const;
  public:
  void clear_bbox_h();
  int32_t bbox_h() const;
  void set_bbox_h(int32_t value);
  private:
  int32_t _internal_bbox_h() const;
  void _internal_set_bbox_h(int32_t value);
  public:

  // optional float bbox_p = 6;
  bool has_bbox_p() const;
  private:
  bool _internal_has_bbox_p() const;
  public:
  void clear_bbox_p();
  float bbox_p() const;
  void set_bbox_p(float value);
  private:
  float _internal_bbox_p() const;
  void _internal_set_bbox_p(float value);
  public:

  // @@protoc_insertion_point(class_scope:offline_bbox.Bbox)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  int32_t obj_id_;
  int32_t bbox_x_;
  int32_t bbox_y_;
  int32_t bbox_w_;
  int32_t bbox_h_;
  float bbox_p_;
  friend struct ::TableStruct_offline_5fbbox_2eproto;
};
// -------------------------------------------------------------------

class offline_bboxes final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:offline_bbox.offline_bboxes) */ {
 public:
  inline offline_bboxes() : offline_bboxes(nullptr) {}
  ~offline_bboxes() override;
  explicit PROTOBUF_CONSTEXPR offline_bboxes(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  offline_bboxes(const offline_bboxes& from);
  offline_bboxes(offline_bboxes&& from) noexcept
    : offline_bboxes() {
    *this = ::std::move(from);
  }

  inline offline_bboxes& operator=(const offline_bboxes& from) {
    CopyFrom(from);
    return *this;
  }
  inline offline_bboxes& operator=(offline_bboxes&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const offline_bboxes& default_instance() {
    return *internal_default_instance();
  }
  static inline const offline_bboxes* internal_default_instance() {
    return reinterpret_cast<const offline_bboxes*>(
               &_offline_bboxes_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(offline_bboxes& a, offline_bboxes& b) {
    a.Swap(&b);
  }
  inline void Swap(offline_bboxes* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(offline_bboxes* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  offline_bboxes* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<offline_bboxes>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const offline_bboxes& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const offline_bboxes& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(offline_bboxes* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "offline_bbox.offline_bboxes";
  }
  protected:
  explicit offline_bboxes(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kBboxFieldNumber = 5,
    kTimeSecFieldNumber = 1,
    kImageDataFieldNumber = 2,
    kSeqNameFieldNumber = 3,
    kUpdateFreqFieldNumber = 4,
  };
  // repeated .offline_bbox.Bbox bbox = 5;
  int bbox_size() const;
  private:
  int _internal_bbox_size() const;
  public:
  void clear_bbox();
  ::offline_bbox::Bbox* mutable_bbox(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::offline_bbox::Bbox >*
      mutable_bbox();
  private:
  const ::offline_bbox::Bbox& _internal_bbox(int index) const;
  ::offline_bbox::Bbox* _internal_add_bbox();
  public:
  const ::offline_bbox::Bbox& bbox(int index) const;
  ::offline_bbox::Bbox* add_bbox();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::offline_bbox::Bbox >&
      bbox() const;

  // optional string time_sec = 1;
  bool has_time_sec() const;
  private:
  bool _internal_has_time_sec() const;
  public:
  void clear_time_sec();
  const std::string& time_sec() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_time_sec(ArgT0&& arg0, ArgT... args);
  std::string* mutable_time_sec();
  PROTOBUF_NODISCARD std::string* release_time_sec();
  void set_allocated_time_sec(std::string* time_sec);
  private:
  const std::string& _internal_time_sec() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_time_sec(const std::string& value);
  std::string* _internal_mutable_time_sec();
  public:

  // optional bytes image_data = 2;
  bool has_image_data() const;
  private:
  bool _internal_has_image_data() const;
  public:
  void clear_image_data();
  const std::string& image_data() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_image_data(ArgT0&& arg0, ArgT... args);
  std::string* mutable_image_data();
  PROTOBUF_NODISCARD std::string* release_image_data();
  void set_allocated_image_data(std::string* image_data);
  private:
  const std::string& _internal_image_data() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_image_data(const std::string& value);
  std::string* _internal_mutable_image_data();
  public:

  // optional string seq_name = 3;
  bool has_seq_name() const;
  private:
  bool _internal_has_seq_name() const;
  public:
  void clear_seq_name();
  const std::string& seq_name() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_seq_name(ArgT0&& arg0, ArgT... args);
  std::string* mutable_seq_name();
  PROTOBUF_NODISCARD std::string* release_seq_name();
  void set_allocated_seq_name(std::string* seq_name);
  private:
  const std::string& _internal_seq_name() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_seq_name(const std::string& value);
  std::string* _internal_mutable_seq_name();
  public:

  // optional string update_freq = 4;
  bool has_update_freq() const;
  private:
  bool _internal_has_update_freq() const;
  public:
  void clear_update_freq();
  const std::string& update_freq() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_update_freq(ArgT0&& arg0, ArgT... args);
  std::string* mutable_update_freq();
  PROTOBUF_NODISCARD std::string* release_update_freq();
  void set_allocated_update_freq(std::string* update_freq);
  private:
  const std::string& _internal_update_freq() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_update_freq(const std::string& value);
  std::string* _internal_mutable_update_freq();
  public:

  // @@protoc_insertion_point(class_scope:offline_bbox.offline_bboxes)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::offline_bbox::Bbox > bbox_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr time_sec_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr image_data_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr seq_name_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr update_freq_;
  friend struct ::TableStruct_offline_5fbbox_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Bbox

// optional int32 obj_id = 1;
inline bool Bbox::_internal_has_obj_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Bbox::has_obj_id() const {
  return _internal_has_obj_id();
}
inline void Bbox::clear_obj_id() {
  obj_id_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline int32_t Bbox::_internal_obj_id() const {
  return obj_id_;
}
inline int32_t Bbox::obj_id() const {
  // @@protoc_insertion_point(field_get:offline_bbox.Bbox.obj_id)
  return _internal_obj_id();
}
inline void Bbox::_internal_set_obj_id(int32_t value) {
  _has_bits_[0] |= 0x00000001u;
  obj_id_ = value;
}
inline void Bbox::set_obj_id(int32_t value) {
  _internal_set_obj_id(value);
  // @@protoc_insertion_point(field_set:offline_bbox.Bbox.obj_id)
}

// optional int32 bbox_x = 2;
inline bool Bbox::_internal_has_bbox_x() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool Bbox::has_bbox_x() const {
  return _internal_has_bbox_x();
}
inline void Bbox::clear_bbox_x() {
  bbox_x_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline int32_t Bbox::_internal_bbox_x() const {
  return bbox_x_;
}
inline int32_t Bbox::bbox_x() const {
  // @@protoc_insertion_point(field_get:offline_bbox.Bbox.bbox_x)
  return _internal_bbox_x();
}
inline void Bbox::_internal_set_bbox_x(int32_t value) {
  _has_bits_[0] |= 0x00000002u;
  bbox_x_ = value;
}
inline void Bbox::set_bbox_x(int32_t value) {
  _internal_set_bbox_x(value);
  // @@protoc_insertion_point(field_set:offline_bbox.Bbox.bbox_x)
}

// optional int32 bbox_y = 3;
inline bool Bbox::_internal_has_bbox_y() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool Bbox::has_bbox_y() const {
  return _internal_has_bbox_y();
}
inline void Bbox::clear_bbox_y() {
  bbox_y_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline int32_t Bbox::_internal_bbox_y() const {
  return bbox_y_;
}
inline int32_t Bbox::bbox_y() const {
  // @@protoc_insertion_point(field_get:offline_bbox.Bbox.bbox_y)
  return _internal_bbox_y();
}
inline void Bbox::_internal_set_bbox_y(int32_t value) {
  _has_bits_[0] |= 0x00000004u;
  bbox_y_ = value;
}
inline void Bbox::set_bbox_y(int32_t value) {
  _internal_set_bbox_y(value);
  // @@protoc_insertion_point(field_set:offline_bbox.Bbox.bbox_y)
}

// optional int32 bbox_w = 4;
inline bool Bbox::_internal_has_bbox_w() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool Bbox::has_bbox_w() const {
  return _internal_has_bbox_w();
}
inline void Bbox::clear_bbox_w() {
  bbox_w_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline int32_t Bbox::_internal_bbox_w() const {
  return bbox_w_;
}
inline int32_t Bbox::bbox_w() const {
  // @@protoc_insertion_point(field_get:offline_bbox.Bbox.bbox_w)
  return _internal_bbox_w();
}
inline void Bbox::_internal_set_bbox_w(int32_t value) {
  _has_bits_[0] |= 0x00000008u;
  bbox_w_ = value;
}
inline void Bbox::set_bbox_w(int32_t value) {
  _internal_set_bbox_w(value);
  // @@protoc_insertion_point(field_set:offline_bbox.Bbox.bbox_w)
}

// optional int32 bbox_h = 5;
inline bool Bbox::_internal_has_bbox_h() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool Bbox::has_bbox_h() const {
  return _internal_has_bbox_h();
}
inline void Bbox::clear_bbox_h() {
  bbox_h_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline int32_t Bbox::_internal_bbox_h() const {
  return bbox_h_;
}
inline int32_t Bbox::bbox_h() const {
  // @@protoc_insertion_point(field_get:offline_bbox.Bbox.bbox_h)
  return _internal_bbox_h();
}
inline void Bbox::_internal_set_bbox_h(int32_t value) {
  _has_bits_[0] |= 0x00000010u;
  bbox_h_ = value;
}
inline void Bbox::set_bbox_h(int32_t value) {
  _internal_set_bbox_h(value);
  // @@protoc_insertion_point(field_set:offline_bbox.Bbox.bbox_h)
}

// optional float bbox_p = 6;
inline bool Bbox::_internal_has_bbox_p() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool Bbox::has_bbox_p() const {
  return _internal_has_bbox_p();
}
inline void Bbox::clear_bbox_p() {
  bbox_p_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline float Bbox::_internal_bbox_p() const {
  return bbox_p_;
}
inline float Bbox::bbox_p() const {
  // @@protoc_insertion_point(field_get:offline_bbox.Bbox.bbox_p)
  return _internal_bbox_p();
}
inline void Bbox::_internal_set_bbox_p(float value) {
  _has_bits_[0] |= 0x00000020u;
  bbox_p_ = value;
}
inline void Bbox::set_bbox_p(float value) {
  _internal_set_bbox_p(value);
  // @@protoc_insertion_point(field_set:offline_bbox.Bbox.bbox_p)
}

// -------------------------------------------------------------------

// offline_bboxes

// optional string time_sec = 1;
inline bool offline_bboxes::_internal_has_time_sec() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool offline_bboxes::has_time_sec() const {
  return _internal_has_time_sec();
}
inline void offline_bboxes::clear_time_sec() {
  time_sec_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& offline_bboxes::time_sec() const {
  // @@protoc_insertion_point(field_get:offline_bbox.offline_bboxes.time_sec)
  return _internal_time_sec();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void offline_bboxes::set_time_sec(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000001u;
 time_sec_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:offline_bbox.offline_bboxes.time_sec)
}
inline std::string* offline_bboxes::mutable_time_sec() {
  std::string* _s = _internal_mutable_time_sec();
  // @@protoc_insertion_point(field_mutable:offline_bbox.offline_bboxes.time_sec)
  return _s;
}
inline const std::string& offline_bboxes::_internal_time_sec() const {
  return time_sec_.Get();
}
inline void offline_bboxes::_internal_set_time_sec(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  time_sec_.Set(value, GetArenaForAllocation());
}
inline std::string* offline_bboxes::_internal_mutable_time_sec() {
  _has_bits_[0] |= 0x00000001u;
  return time_sec_.Mutable(GetArenaForAllocation());
}
inline std::string* offline_bboxes::release_time_sec() {
  // @@protoc_insertion_point(field_release:offline_bbox.offline_bboxes.time_sec)
  if (!_internal_has_time_sec()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  auto* p = time_sec_.Release();
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (time_sec_.IsDefault()) {
    time_sec_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  return p;
}
inline void offline_bboxes::set_allocated_time_sec(std::string* time_sec) {
  if (time_sec != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  time_sec_.SetAllocated(time_sec, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (time_sec_.IsDefault()) {
    time_sec_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:offline_bbox.offline_bboxes.time_sec)
}

// optional bytes image_data = 2;
inline bool offline_bboxes::_internal_has_image_data() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool offline_bboxes::has_image_data() const {
  return _internal_has_image_data();
}
inline void offline_bboxes::clear_image_data() {
  image_data_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000002u;
}
inline const std::string& offline_bboxes::image_data() const {
  // @@protoc_insertion_point(field_get:offline_bbox.offline_bboxes.image_data)
  return _internal_image_data();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void offline_bboxes::set_image_data(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000002u;
 image_data_.SetBytes(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:offline_bbox.offline_bboxes.image_data)
}
inline std::string* offline_bboxes::mutable_image_data() {
  std::string* _s = _internal_mutable_image_data();
  // @@protoc_insertion_point(field_mutable:offline_bbox.offline_bboxes.image_data)
  return _s;
}
inline const std::string& offline_bboxes::_internal_image_data() const {
  return image_data_.Get();
}
inline void offline_bboxes::_internal_set_image_data(const std::string& value) {
  _has_bits_[0] |= 0x00000002u;
  image_data_.Set(value, GetArenaForAllocation());
}
inline std::string* offline_bboxes::_internal_mutable_image_data() {
  _has_bits_[0] |= 0x00000002u;
  return image_data_.Mutable(GetArenaForAllocation());
}
inline std::string* offline_bboxes::release_image_data() {
  // @@protoc_insertion_point(field_release:offline_bbox.offline_bboxes.image_data)
  if (!_internal_has_image_data()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000002u;
  auto* p = image_data_.Release();
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (image_data_.IsDefault()) {
    image_data_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  return p;
}
inline void offline_bboxes::set_allocated_image_data(std::string* image_data) {
  if (image_data != nullptr) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  image_data_.SetAllocated(image_data, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (image_data_.IsDefault()) {
    image_data_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:offline_bbox.offline_bboxes.image_data)
}

// optional string seq_name = 3;
inline bool offline_bboxes::_internal_has_seq_name() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool offline_bboxes::has_seq_name() const {
  return _internal_has_seq_name();
}
inline void offline_bboxes::clear_seq_name() {
  seq_name_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000004u;
}
inline const std::string& offline_bboxes::seq_name() const {
  // @@protoc_insertion_point(field_get:offline_bbox.offline_bboxes.seq_name)
  return _internal_seq_name();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void offline_bboxes::set_seq_name(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000004u;
 seq_name_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:offline_bbox.offline_bboxes.seq_name)
}
inline std::string* offline_bboxes::mutable_seq_name() {
  std::string* _s = _internal_mutable_seq_name();
  // @@protoc_insertion_point(field_mutable:offline_bbox.offline_bboxes.seq_name)
  return _s;
}
inline const std::string& offline_bboxes::_internal_seq_name() const {
  return seq_name_.Get();
}
inline void offline_bboxes::_internal_set_seq_name(const std::string& value) {
  _has_bits_[0] |= 0x00000004u;
  seq_name_.Set(value, GetArenaForAllocation());
}
inline std::string* offline_bboxes::_internal_mutable_seq_name() {
  _has_bits_[0] |= 0x00000004u;
  return seq_name_.Mutable(GetArenaForAllocation());
}
inline std::string* offline_bboxes::release_seq_name() {
  // @@protoc_insertion_point(field_release:offline_bbox.offline_bboxes.seq_name)
  if (!_internal_has_seq_name()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000004u;
  auto* p = seq_name_.Release();
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (seq_name_.IsDefault()) {
    seq_name_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  return p;
}
inline void offline_bboxes::set_allocated_seq_name(std::string* seq_name) {
  if (seq_name != nullptr) {
    _has_bits_[0] |= 0x00000004u;
  } else {
    _has_bits_[0] &= ~0x00000004u;
  }
  seq_name_.SetAllocated(seq_name, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (seq_name_.IsDefault()) {
    seq_name_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:offline_bbox.offline_bboxes.seq_name)
}

// optional string update_freq = 4;
inline bool offline_bboxes::_internal_has_update_freq() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool offline_bboxes::has_update_freq() const {
  return _internal_has_update_freq();
}
inline void offline_bboxes::clear_update_freq() {
  update_freq_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000008u;
}
inline const std::string& offline_bboxes::update_freq() const {
  // @@protoc_insertion_point(field_get:offline_bbox.offline_bboxes.update_freq)
  return _internal_update_freq();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void offline_bboxes::set_update_freq(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000008u;
 update_freq_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:offline_bbox.offline_bboxes.update_freq)
}
inline std::string* offline_bboxes::mutable_update_freq() {
  std::string* _s = _internal_mutable_update_freq();
  // @@protoc_insertion_point(field_mutable:offline_bbox.offline_bboxes.update_freq)
  return _s;
}
inline const std::string& offline_bboxes::_internal_update_freq() const {
  return update_freq_.Get();
}
inline void offline_bboxes::_internal_set_update_freq(const std::string& value) {
  _has_bits_[0] |= 0x00000008u;
  update_freq_.Set(value, GetArenaForAllocation());
}
inline std::string* offline_bboxes::_internal_mutable_update_freq() {
  _has_bits_[0] |= 0x00000008u;
  return update_freq_.Mutable(GetArenaForAllocation());
}
inline std::string* offline_bboxes::release_update_freq() {
  // @@protoc_insertion_point(field_release:offline_bbox.offline_bboxes.update_freq)
  if (!_internal_has_update_freq()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000008u;
  auto* p = update_freq_.Release();
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (update_freq_.IsDefault()) {
    update_freq_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  return p;
}
inline void offline_bboxes::set_allocated_update_freq(std::string* update_freq) {
  if (update_freq != nullptr) {
    _has_bits_[0] |= 0x00000008u;
  } else {
    _has_bits_[0] &= ~0x00000008u;
  }
  update_freq_.SetAllocated(update_freq, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (update_freq_.IsDefault()) {
    update_freq_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:offline_bbox.offline_bboxes.update_freq)
}

// repeated .offline_bbox.Bbox bbox = 5;
inline int offline_bboxes::_internal_bbox_size() const {
  return bbox_.size();
}
inline int offline_bboxes::bbox_size() const {
  return _internal_bbox_size();
}
inline void offline_bboxes::clear_bbox() {
  bbox_.Clear();
}
inline ::offline_bbox::Bbox* offline_bboxes::mutable_bbox(int index) {
  // @@protoc_insertion_point(field_mutable:offline_bbox.offline_bboxes.bbox)
  return bbox_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::offline_bbox::Bbox >*
offline_bboxes::mutable_bbox() {
  // @@protoc_insertion_point(field_mutable_list:offline_bbox.offline_bboxes.bbox)
  return &bbox_;
}
inline const ::offline_bbox::Bbox& offline_bboxes::_internal_bbox(int index) const {
  return bbox_.Get(index);
}
inline const ::offline_bbox::Bbox& offline_bboxes::bbox(int index) const {
  // @@protoc_insertion_point(field_get:offline_bbox.offline_bboxes.bbox)
  return _internal_bbox(index);
}
inline ::offline_bbox::Bbox* offline_bboxes::_internal_add_bbox() {
  return bbox_.Add();
}
inline ::offline_bbox::Bbox* offline_bboxes::add_bbox() {
  ::offline_bbox::Bbox* _add = _internal_add_bbox();
  // @@protoc_insertion_point(field_add:offline_bbox.offline_bboxes.bbox)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::offline_bbox::Bbox >&
offline_bboxes::bbox() const {
  // @@protoc_insertion_point(field_list:offline_bbox.offline_bboxes.bbox)
  return bbox_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace offline_bbox

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_offline_5fbbox_2eproto
