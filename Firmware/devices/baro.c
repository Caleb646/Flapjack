#include "devices/baro.h"

#include "drivers/baro/barodrv.h"

#include <math.h>
#include <string.h>

/*
 * Hypsometric altitude above a pressure datum:
 *
 *      h = (T0 / L) * (1 - (P/P0)^(1/5.255))
 *
 * referencePa/referenceTempC are the DATUM - the pressure and temperature
 * measured where altitude should read zero - so the result is height above that
 * point. Pass 101325 / 15.0 to get ISA pressure altitude instead.
 *
 * The familiar "44330" constant is not a constant at all: 288.15 K / 0.0065 K/m
 * IS 44330, i.e. the sea-level special case of T0/L. Using it at a datum that
 * is not at sea level leaves a pure SCALE error, because the air column being
 * measured is colder and therefore denser than the formula assumes. Substituting
 * the measured datum temperature removes it exactly - verified to below a
 * millimetre from 0 to 2000 m of field elevation, where the fixed constant was
 * over-reading by up to 4.7 %.
 *
 * THE TRADE THIS MAKES. Accuracy now depends on the temperature reading instead
 * of on field elevation, and a datum-temperature error of dT costs dT/T0, about
 * 0.35 % per degree C. Break-even against the old fixed constant:
 *
 *      field elevation     old error     temperature budget
 *          100 m             0.23 %          0.65 C
 *          500 m             1.14 %          3.25 C
 *         1000 m             2.31 %          6.50 C
 *
 * So this is a clear win at altitude and a WASH near sea level, and it becomes a
 * net loss if the temperature is off by more than the right-hand column. That
 * matters because a barometer reports its own DIE temperature, not ambient, and
 * a part sitting in a warm enclosure reads high. Two things keep it honest here:
 * the datum is taken at boot, when the board is closest to ambient, and the SIL
 * cannot see this failure at all (JSBSim reports true air temperature), so do
 * not read a clean SIL result as evidence the sensor is good enough.
 */
float Baro_PressureToAltitude (float pressurePa, float referencePa, float referenceTempC) {

    /* powf of a non-positive base is a domain error, and a zero reference is a
     * divide by zero. Both mean the datum or the sample is garbage rather than
     * that the vehicle is at some altitude, so report the datum height and let
     * the caller's validity flag carry the doubt. */
    if (pressurePa <= 0.0F || referencePa <= 0.0F) {
        return 0.0F;
    }

    /* Below absolute zero is a dead or unread sensor, not cold weather. Fall
     * back to the ISA sea-level value rather than returning nonsense: that is
     * the behaviour this function had before it took a temperature at all. */
    float kelvin = referenceTempC + 273.15F;
    if (kelvin <= 1.0F) {
        kelvin = 288.15F;
    }

    return (kelvin / 0.0065F) * (1.0F - powf (pressurePa / referencePa, 1.0F / 5.255F));
}

eSTATUS_t Baro_Init (Baro_t* pOutSensor, DataReadySignal_t const* pSignal) {

    if (!pOutSensor) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutSensor, 0, sizeof (Baro_t));

    if (pSignal) {
        pOutSensor->drv.cfg.signal = *pSignal;
    }

    return BaroDrv_Init (&pOutSensor->drv);
}

eSTATUS_t Baro_Update (Baro_t* pSensor) {

    if (!pSensor) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status =
    pSensor->drv.Read (pSensor->drv.ctx, false, &pSensor->data);
    if (STATUS_FAIL (status)) {
        return status;
    }

    pSensor->usLastUpdate = GetMicroseconds ();
    return eSTATUS_SUCCESS;
}
