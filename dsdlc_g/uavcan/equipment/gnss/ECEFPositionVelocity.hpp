/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /cygdrive/c/dev/twin-engine-controller/libuavcan/dsdl/uavcan/equipment/gnss/ECEFPositionVelocity.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_GNSS_ECEFPOSITIONVELOCITY_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_GNSS_ECEFPOSITIONVELOCITY_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Nested type.
# GNSS ECEF high resolution position and velocity.
#
# ECEF is an acronym for Earth-Centered-Earth-Fixed, which is a cartesian
# coordinate system which rotates with the earth. The origin (0,0,0) is
# located at the center of the earth. The x-axis is a vector pointing from
# the origin with positive direction towards 0 degrees latitude and
# longitude (equator, at the prime meridian). The z-axis is a vector
# pointing from the origin towards the north-pole. The y-axis completes a
# right-handed coordinate system.
#

float32[3] velocity_xyz            # XYZ velocity in m/s

int36[3] position_xyz_mm           # XYZ-axis coordinates in mm

void6                              # Aligns the following array at byte boundary

float16[<=36] covariance           # Position and velocity covariance in the ECEF frame. Units are m^2 for position,
                                   # (m/s)^2 for velocity, and m^2/s for position/velocity.
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.gnss.ECEFPositionVelocity
saturated float32[3] velocity_xyz
saturated int36[3] position_xyz_mm
void6
saturated float16[<=36] covariance
******************************************************************************/

#undef velocity_xyz
#undef position_xyz_mm
#undef _void_0
#undef covariance

namespace uavcan
{
namespace equipment
{
namespace gnss
{

template <int _tmpl>
struct UAVCAN_EXPORT ECEFPositionVelocity_
{
    typedef const ECEFPositionVelocity_<_tmpl>& ParameterType;
    typedef ECEFPositionVelocity_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > velocity_xyz;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 36, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > position_xyz_mm;
        typedef ::uavcan::IntegerSpec< 6, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > _void_0;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 36 > covariance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::velocity_xyz::MinBitLen
            + FieldTypes::position_xyz_mm::MinBitLen
            + FieldTypes::_void_0::MinBitLen
            + FieldTypes::covariance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::velocity_xyz::MaxBitLen
            + FieldTypes::position_xyz_mm::MaxBitLen
            + FieldTypes::_void_0::MaxBitLen
            + FieldTypes::covariance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::velocity_xyz >::Type velocity_xyz;
    typename ::uavcan::StorageType< typename FieldTypes::position_xyz_mm >::Type position_xyz_mm;
    typename ::uavcan::StorageType< typename FieldTypes::covariance >::Type covariance;

    ECEFPositionVelocity_()
        : velocity_xyz()
        , position_xyz_mm()
        , covariance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<792 == MaxBitLen>::check();
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
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.gnss.ECEFPositionVelocity";
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
bool ECEFPositionVelocity_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        velocity_xyz == rhs.velocity_xyz &&
        position_xyz_mm == rhs.position_xyz_mm &&
        covariance == rhs.covariance;
}

template <int _tmpl>
bool ECEFPositionVelocity_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(velocity_xyz, rhs.velocity_xyz) &&
        ::uavcan::areClose(position_xyz_mm, rhs.position_xyz_mm) &&
        ::uavcan::areClose(covariance, rhs.covariance);
}

template <int _tmpl>
int ECEFPositionVelocity_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    typename ::uavcan::StorageType< typename FieldTypes::_void_0 >::Type _void_0 = 0;
    int res = 1;
    res = FieldTypes::velocity_xyz::encode(self.velocity_xyz, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::position_xyz_mm::encode(self.position_xyz_mm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::_void_0::encode(_void_0, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::covariance::encode(self.covariance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int ECEFPositionVelocity_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    typename ::uavcan::StorageType< typename FieldTypes::_void_0 >::Type _void_0 = 0;
    int res = 1;
    res = FieldTypes::velocity_xyz::decode(self.velocity_xyz, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::position_xyz_mm::decode(self.position_xyz_mm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::_void_0::decode(_void_0, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::covariance::decode(self.covariance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature ECEFPositionVelocity_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x24A5DA4ABEE3A248ULL);

    FieldTypes::velocity_xyz::extendDataTypeSignature(signature);
    FieldTypes::position_xyz_mm::extendDataTypeSignature(signature);
    FieldTypes::_void_0::extendDataTypeSignature(signature);
    FieldTypes::covariance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef ECEFPositionVelocity_<0> ECEFPositionVelocity;

// No default registration

} // Namespace gnss
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::gnss::ECEFPositionVelocity >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::gnss::ECEFPositionVelocity::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::gnss::ECEFPositionVelocity >::stream(Stream& s, ::uavcan::equipment::gnss::ECEFPositionVelocity::ParameterType obj, const int level)
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
    s << "velocity_xyz: ";
    YamlStreamer< ::uavcan::equipment::gnss::ECEFPositionVelocity::FieldTypes::velocity_xyz >::stream(s, obj.velocity_xyz, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "position_xyz_mm: ";
    YamlStreamer< ::uavcan::equipment::gnss::ECEFPositionVelocity::FieldTypes::position_xyz_mm >::stream(s, obj.position_xyz_mm, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "covariance: ";
    YamlStreamer< ::uavcan::equipment::gnss::ECEFPositionVelocity::FieldTypes::covariance >::stream(s, obj.covariance, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace gnss
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::gnss::ECEFPositionVelocity::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::gnss::ECEFPositionVelocity >::stream(s, obj, 0);
    return s;
}

} // Namespace gnss
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_GNSS_ECEFPOSITIONVELOCITY_HPP_INCLUDED