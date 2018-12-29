/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /cygdrive/c/dev/twin-engine-controller/libuavcan/dsdl/uavcan/protocol/5.Panic.uavcan
 */

#ifndef UAVCAN_PROTOCOL_PANIC_HPP_INCLUDED
#define UAVCAN_PROTOCOL_PANIC_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# This message may be published periodically to inform network participants that the system has encountered
# an unrecoverable fault and is not capable of further operation.
#
# Nodes that are expected to react to this message should wait for at least MIN_MESSAGES subsequent messages
# with any reason text from any sender published with the interval no higher than MAX_INTERVAL_MS before
# undertaking any emergency actions.
#

uint8 MIN_MESSAGES = 3

uint16 MAX_INTERVAL_MS = 500

#
# Short description that would fit a single CAN frame.
#
uint8[<=7] reason_text
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.Panic
saturated uint8[<=7] reason_text
******************************************************************************/

#undef reason_text
#undef MIN_MESSAGES
#undef MAX_INTERVAL_MS

namespace uavcan
{
namespace protocol
{

template <int _tmpl>
struct UAVCAN_EXPORT Panic_
{
    typedef const Panic_<_tmpl>& ParameterType;
    typedef Panic_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MIN_MESSAGES;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MAX_INTERVAL_MS;
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 7 > reason_text;
    };

    enum
    {
        MinBitLen
            = FieldTypes::reason_text::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::reason_text::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::MIN_MESSAGES >::Type MIN_MESSAGES; // 3
    static const typename ::uavcan::StorageType< typename ConstantTypes::MAX_INTERVAL_MS >::Type MAX_INTERVAL_MS; // 500

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::reason_text >::Type reason_text;

    Panic_()
        : reason_text()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<59 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 5 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.Panic";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool Panic_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        reason_text == rhs.reason_text;
}

template <int _tmpl>
bool Panic_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(reason_text, rhs.reason_text);
}

template <int _tmpl>
int Panic_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::reason_text::encode(self.reason_text, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Panic_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::reason_text::decode(self.reason_text, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Panic_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x8B79B4101811C1D7ULL);

    FieldTypes::reason_text::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename Panic_<_tmpl>::ConstantTypes::MIN_MESSAGES >::Type
    Panic_<_tmpl>::MIN_MESSAGES = 3U; // 3

template <int _tmpl>
const typename ::uavcan::StorageType< typename Panic_<_tmpl>::ConstantTypes::MAX_INTERVAL_MS >::Type
    Panic_<_tmpl>::MAX_INTERVAL_MS = 500U; // 500

/*
 * Final typedef
 */
typedef Panic_<0> Panic;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::Panic > _uavcan_gdtr_registrator_Panic;

}

} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::Panic >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::Panic::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::Panic >::stream(Stream& s, ::uavcan::protocol::Panic::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "reason_text: ";
    YamlStreamer< ::uavcan::protocol::Panic::FieldTypes::reason_text >::stream(s, obj.reason_text, level + 1);
}

}

namespace uavcan
{
namespace protocol
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::Panic::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::Panic >::stream(s, obj, 0);
    return s;
}

} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_PANIC_HPP_INCLUDED