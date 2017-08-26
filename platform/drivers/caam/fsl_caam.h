/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_CAAM_H_
#define _FSL_CAAM_H_

#if 0
#include "fsl_common.h"
#else

/* Prototype build on Freescale MQX for Vybrid */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* temporary until CAAM peripheral module memory map is available in Kinetis SDK SoC files */
typedef uint32_t CAAM_Type;
typedef int32_t status_t;
#define CAAM_BASE ((CAAM_Type *)0x400F0000u)
#define CAAM_MCR_REG (*(volatile uint32_t *)((uint8_t *)CAAM_BASE + 0x0004u))

/*! @brief Construct a status code value from a group and code number. */
#define MAKE_STATUS(group, code) ((((group)*100) + (code)))

/*! @brief Status group numbers. */
enum _status_groups
{
    kStatusGroup_Generic = 0, /*!< Group number for generic status codes. */
};

/*! @brief Generic status return codes. */
enum _generic_status
{
    kStatus_Success = MAKE_STATUS(kStatusGroup_Generic, 0),
    kStatus_Fail = MAKE_STATUS(kStatusGroup_Generic, 1),
    kStatus_ReadOnly = MAKE_STATUS(kStatusGroup_Generic, 2),
    kStatus_OutOfRange = MAKE_STATUS(kStatusGroup_Generic, 3),
    kStatus_InvalidArgument = MAKE_STATUS(kStatusGroup_Generic, 4),
    kStatus_Timeout = MAKE_STATUS(kStatusGroup_Generic, 5),
    kStatus_NoTransferInProgress = MAKE_STATUS(kStatusGroup_Generic, 6),
    kStatus_Again = MAKE_STATUS(kStatusGroup_Generic, 7),
};

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#include "mqx.h"

static inline uint32_t DisableGlobalIRQ(void)
{
    _int_disable();
    return 0;
}

static inline void EnableGlobalIRQ(uint32_t currPriMask)
{
    _int_enable();
    return;
}
#endif

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*!
 * @addtogroup caam_driver
 * @{
 */
/*! @file */
/*! @name Driver version */
/*@{*/
/*! @brief CAAM driver version. Version 2.0.0.
 *
 * Current version: 2.0.0
 *
 * Change log:
 * - Version 2.0.0
 *   - Initial version
 */
#define FSL_CAAM_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/
/*! @} */

/*! @brief CAAM callback function. */
typedef struct _caam_job_callback
{
    void (*JobCompleted)(void *userData); /*!< CAAM Job complete callback */
} caam_job_callback_t;

/*! @brief CAAM job ring selection. */
typedef enum _caam_job_ring_t
{
    kCAAM_JobRing0 = 0u, /*!< CAAM Job ring 0 */
    kCAAM_JobRing1 = 1u, /*!< CAAM Job ring 1 */
} caam_job_ring_t;

/*! @brief CAAM handle
 *  Specifies jobRing and optionally the user callback function.
 *  The user callback functions is invoked only if jobRing interrupt has been enabled by the user.
 *  By default the jobRing interrupt is disabled (default job complete test is polling CAAM output ring).
 */
typedef struct _caam_handle_t
{
    caam_job_ring_t jobRing;

    /* Callback functions */
    caam_job_callback_t callback; /*!< Callback function */
    void *userData;               /*!< Parameter for CAAM job complete callback */
} caam_handle_t;

/*! @brief CAAM driver wait mechanism
 */
typedef enum _caam_wait_mode
{
    kCAAM_Blocking = 0u,    /*!< CAAM_Wait blocking mode */
    kCAAM_Nonblocking = 1u, /*!< CAAM Wait non-blocking mode */
} caam_wait_mode_t;

/*! @brief Memory buffer to hold CAAM descriptor for AESA ECB job */
typedef uint32_t caam_desc_aes_ecb_t[8];

/*! @brief Memory buffer to hold CAAM descriptor for AESA CBC job */
typedef uint32_t caam_desc_aes_cbc_t[10];

/*! @brief Memory buffer to hold CAAM descriptor for AESA CTR job */
typedef uint32_t caam_desc_aes_ctr_t[23];

/*! @brief Memory buffer to hold CAAM descriptor for AESA CCM job */
typedef uint32_t caam_desc_aes_ccm_t[24];

/*! @brief Memory buffer to hold CAAM descriptor for AESA GCM job */
typedef uint32_t caam_desc_aes_gcm_t[24];

/*! @brief Memory buffer to hold CAAM descriptor for MDHA job or AESA CMAC job */
typedef uint32_t caam_desc_hash_t[21];

/*! @brief Memory buffer to hold CAAM descriptor for RNG jobs */
typedef uint32_t caam_desc_rng_t[6];

/*! @brief Memory buffer to hold CAAM descriptor for DESA jobs */
typedef uint32_t caam_desc_cipher_des_t[15];

/*******************************************************************************
 * AES Definitions
 *******************************************************************************/

/*!
 * @addtogroup caam_driver_aes
 * @{
 */

/*! AES block size in bytes */
#define CAAM_AES_BLOCK_SIZE 16

/*! AESA XCBC-MAC or CMAC request CLASS 1 (default) or CLASS 2 CHA */
#ifndef CAAM_AES_MAC_CLASS
#define CAAM_AES_MAC_CLASS 0x02000000u
#endif

/*!
 *@}
 */ /* end of caam_driver_aes */

/*******************************************************************************
 * DES Definitions
 *******************************************************************************/
/*!
 * @addtogroup caam_driver_des
 * @{
 */

/*! @brief CAAM DES key size - 64 bits. */
#define CAAM_DES_KEY_SIZE 8

/*! @brief CAAM DES IV size - 8 bytes */
#define CAAM_DES_IV_SIZE 8

/*!
 *@}
 */ /* end of caam_driver_des */

/*******************************************************************************
 * HASH Definitions
 ******************************************************************************/
/*!
 * @addtogroup caam_driver_hash
 * @{
 */

/*! @brief Supported cryptographic block cipher functions for HASH creation */
typedef enum _caam_hash_algo_t
{
    kCAAM_XcbcMac = 0, /*!< XCBC-MAC (AES engine) */
    kCAAM_Cmac,        /*!< CMAC (AES engine) */
    kCAAM_Sha1,        /*!< SHA_1   (MDHA engine)  */
    kCAAM_Sha224,      /*!< SHA_224 (MDHA engine)  */
    kCAAM_Sha256,      /*!< SHA_256 (MDHA engine)  */
} caam_hash_algo_t;

/*! @brief CAAM HASH Context size. */
#define CAAM_SHA_BLOCK_SIZE 64                   /*!< SHA-1, SHA-224 & SHA-256 block size  */
#define CAAM_HASH_BLOCK_SIZE CAAM_SHA_BLOCK_SIZE /*!< CAAM hash block size  */

/*! @brief CAAM HASH Context size. */
#define CAAM_HASH_CTX_SIZE 42

/*! @brief Storage type used to save hash context. */
typedef uint32_t caam_hash_ctx_t[CAAM_HASH_CTX_SIZE];

/*!
 *@}
 */ /* end of caam_driver_hash */

/*******************************************************************************
 * RNG Definitions
 ******************************************************************************/
/*!
 * @addtogroup caam_driver_rng
 * @{
 */

/*! @brief CAAM RNG state handle */
typedef enum _caam_rng_state_handle
{
    kCAAM_RngStateHandle0 = 0u, /*!< CAAM RNG state handle 0 */
    kCAAM_RngStateHandle1 = 1u, /*!< CAAM RNG state handle 1 */
} caam_rng_state_handle_t;

/*! @brief Type of random data to generate */
typedef enum _caam_rng_random_type
{
    kCAAM_RngDataAny = 0u,       /*!< CAAM RNG any random data bytes */
    kCAAM_RngDataOddParity = 1u, /*!< CAAM RNG odd parity random data bytes */
    kCAAM_RngDataNonZero = 2u,   /*!< CAAM RNG non zero random data bytes */
} caam_rng_random_type_t;

/*! @brief 256-bit value used as optional additional entropy input */
typedef uint32_t caam_rng_generic256_t[256 / sizeof(uint32_t)];

/*! @brief CAAM RNG configuration */
typedef struct _caam_rng_user_config
{
    uint32_t autoReseedInterval; /*!< Automatic reseed internal. If set to zero, CAAM RNG will use hardware default
                                      interval of 10.000.000 generate requests. */
    caam_rng_generic256_t *personalString; /*!< NULL or pointer to optional personalization string */
} caam_rng_config_t;

/*!
 *@}
 */ /* end of caam_driver_rng */

/*******************************************************************************
 * PKHA Definitions
 ******************************************************************************/
/*!
 * @addtogroup caam_driver_pkha
 * @{
 */
/*! PKHA ECC point structure */
typedef struct _caam_pkha_ecc_point_t
{
    uint8_t *X; /*!< X coordinate (affine) */
    uint8_t *Y; /*!< Y coordinate (affine) */
} caam_pkha_ecc_point_t;

/*! @brief Use of timing equalized version of a PKHA function. */
typedef enum _caam_pkha_timing_t
{
    kCAAM_PKHA_NoTimingEqualized = 0U, /*!< Normal version of a PKHA operation */
    kCAAM_PKHA_TimingEqualized = 1U    /*!< Timing-equalized version of a PKHA operation  */
} caam_pkha_timing_t;

/*! @brief Integer vs binary polynomial arithmetic selection. */
typedef enum _caam_pkha_f2m_t
{
    kCAAM_PKHA_IntegerArith = 0U, /*!< Use integer arithmetic */
    kCAAM_PKHA_F2mArith = 1U      /*!< Use binary polynomial arithmetic */
} caam_pkha_f2m_t;

/*! @brief Montgomery or normal PKHA input format. */
typedef enum _caam_pkha_montgomery_form_t
{
    kCAAM_PKHA_NormalValue = 0U,     /*!< PKHA number is normal integer */
    kCAAM_PKHA_MontgomeryFormat = 1U /*!< PKHA number is in montgomery format */
} caam_pkha_montgomery_form_t;

/*!
 *@}
 */ /* end of caam_driver_pkha */

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @addtogroup caam_driver
 * @{
 */

/*!
 * @brief Initializes the CAAM driver.
 * This function initializes the CAAM driver.
 * @param base CAAM peripheral base address
 */
void CAAM_Init(CAAM_Type *base);

/*!
 * @brief Deinitializes the CAAM driver.
 * This function deinitializes the CAAM driver.
 * @param base CAAM peripheral base address
 */
void CAAM_Deinit(CAAM_Type *base);

/*!
 * @brief Wait for a CAAM job to complete.
 *
 * This function polls CAAM output ring for a specific job.
 *
 * The CAAM job ring is specified by the jobRing field in the caam_handle_t structure.
 * The job to be waited is specified by it's descriptor address.
 *
 * This function has two modes, determined by the mode argument.
 * In blocking mode, the function polls the specified jobRing until the descriptor
 * is available in the CAAM output job ring.
 * In non-blocking mode, it polls the output ring once and returns status
 * immediately.
 *
 * The function can be called from multiple threads or interrupt service routines,
 * as internally it uses global critical section (global interrupt disable enable)
 * to protect it's operation against concurrent accesses.
 * The global interrupt is disabled only when the descriptor is found
 * in the output ring, for a very short time, to remove the descriptor from the output ring
 * safely.
 *
 * @param base CAAM peripheral base address
 * @param handle Data structure with CAAM jobRing used for this request
 * @param descriptor
 * @param mode Blocking and non-blocking mode. Zero is blocking. Non-zero is non-blocking.
 * @return kStatus_Success the CAAM job has completed with zero job termination status word
 * @return kStatus_Fail the CAAM job has completed with non-zero job termination status word
 * @return kStatus_Again In non-blocking mode, the job is not ready in the CAAM Output Ring
 */
status_t CAAM_Wait(CAAM_Type *base, caam_handle_t *handle, void *descriptor, caam_wait_mode_t mode);

/*!
 * @brief Writes to JRMID 64-bit register
 *
 * Writes given values to JRMID 64-bit register.
 * The Job Ring MID Register is used to indicate
 * the Security Domain that currently owns the job ring and to specify the MID values that
 * the CAAM DMA asserts when reading or writing memory on behalf of descriptors
 * fetched from a particular job ring. The register is also used to specify the IP Register Bus
 * MID and TrustZone ns signal value that must be asserted by the processor in order to
 * read or write the registers that are specific to a particular job ring.
 * This register also contains a bit that grants permission for Trusted Descriptors to be
 * created in this job ring.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @return Status from the register write operation.
 */
status_t CAAM_SetJobRingMID(CAAM_Type *base, caam_handle_t *handle, uint32_t highWord, uint32_t lowWord);

/*******************************************************************************
 * AES API
 ******************************************************************************/

/*!
 * @addtogroup caam_driver_aes
 * @{
 */

/*!
 * @brief Encrypts AES using the ECB block mode.
 *
 * Encrypts AES using the ECB block mode.
 *
 * @param base CAAM peripheral base address
 * @param plaintext Input plain text to encrypt
 * @param[out] ciphertext Output cipher text
 * @param size Size of input and output data in bytes. Must be multiple of 16 bytes.
 * @param key Input key to use for encryption
 * @param keySize Size of the input key, in bytes. Must be 16, 24, or 32.
 * @return Status from encrypt operation
 */
status_t CAAM_AES_EncryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t *key,
                             uint32_t keySize);

status_t CAAM_AES_EncryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_ecb_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t *key,
                                        uint32_t keySize);

/*!
 * @brief Decrypts AES using ECB block mode.
 *
 * Decrypts AES using ECB block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input cipher text to decrypt
 * @param[out] plaintext Output plain text
 * @param size Size of input and output data in bytes. Must be multiple of 16 bytes.
 * @param key Input key.
 * @param keySize Size of the input key, in bytes. Must be 16, 24, or 32.
 * @return Status from decrypt operation
 */
status_t CAAM_AES_DecryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t *key,
                             uint32_t keySize);

status_t CAAM_AES_DecryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_ecb_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t *key,
                                        uint32_t keySize);

/*!
 * @brief Encrypts AES using CBC block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plain text to encrypt
 * @param[out] ciphertext Output cipher text
 * @param size Size of input and output data in bytes. Must be multiple of 16 bytes.
 * @param iv Input initial vector to combine with the first input block.
 * @param key Input key to use for encryption
 * @param keySize Size of the input key, in bytes. Must be 16, 24, or 32.
 * @return Status from encrypt operation
 */
status_t CAAM_AES_EncryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t iv[CAAM_AES_BLOCK_SIZE],
                             const uint8_t *key,
                             uint32_t keySize);

status_t CAAM_AES_EncryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_cbc_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t *iv,
                                        const uint8_t *key,
                                        uint32_t keySize);

/*!
 * @brief Decrypts AES using CBC block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input cipher text to decrypt
 * @param[out] plaintext Output plain text
 * @param size Size of input and output data in bytes. Must be multiple of 16 bytes.
 * @param iv Input initial vector to combine with the first input block.
 * @param key Input key to use for decryption
 * @param keySize Size of the input key, in bytes. Must be 16, 24, or 32.
 * @return Status from decrypt operation
 */
status_t CAAM_AES_DecryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t iv[CAAM_AES_BLOCK_SIZE],
                             const uint8_t *key,
                             uint32_t keySize);

status_t CAAM_AES_DecryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_cbc_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t *iv,
                                        const uint8_t *key,
                                        uint32_t keySize);

/*!
 * @brief Encrypts or decrypts AES using CTR block mode.
 *
 * Encrypts or decrypts AES using CTR block mode.
 * AES CTR mode uses only forward AES cipher and same algorithm for encryption and decryption.
 * The only difference between encryption and decryption is that, for encryption, the input argument
 * is plain text and the output argument is cipher text. For decryption, the input argument is cipher text
 * and the output argument is plain text.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param input Input data for CTR block mode
 * @param[out] output Output data for CTR block mode
 * @param size Size of input and output data in bytes
 * @param[in,out] counter Input counter (updates on return)
 * @param key Input key to use for forward AES cipher
 * @param keySize Size of the input key, in bytes. Must be 16, 24, or 32.
 * @param[out] counterlast Output cipher of last counter, for chained CTR calls. NULL can be passed if chained calls are
 * not used.
 * @param[out] szLeft Output number of bytes in left unused in counterlast block. NULL can be passed if chained calls
 * are not used.
 * @return Status from encrypt operation
 */
status_t CAAM_AES_CryptCtr(CAAM_Type *base,
                           caam_handle_t *handle,
                           const uint8_t *input,
                           uint8_t *output,
                           uint32_t size,
                           uint8_t counter[CAAM_AES_BLOCK_SIZE],
                           const uint8_t *key,
                           uint32_t keySize,
                           uint8_t counterlast[CAAM_AES_BLOCK_SIZE],
                           uint32_t *szLeft);

status_t CAAM_AES_CryptCtrNonBlocking(CAAM_Type *base,
                                      caam_handle_t *handle,
                                      caam_desc_aes_ctr_t descriptor,
                                      const uint8_t *input,
                                      uint8_t *output,
                                      uint32_t size,
                                      uint8_t *counter,
                                      const uint8_t *key,
                                      uint32_t keySize,
                                      uint8_t *counterlast,
                                      uint32_t *szLeft);

/*!
 * @brief Encrypts AES and tags using CCM block mode.
 *
 * Encrypts AES and optionally tags using CCM block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plain text to encrypt
 * @param[out] ciphertext Output cipher text.
 * @param size Size of input and output data in bytes. Zero means authentication only.
 * @param iv Nonce
 * @param ivSize Length of the Nonce in bytes. Must be 7, 8, 9, 10, 11, 12, or 13.
 * @param aad Input additional authentication data. Can be NULL if aadSize is zero.
 * @param aadSize Input size in bytes of AAD. Zero means data mode only (authentication skipped).
 * @param key Input key to use for encryption
 * @param keySize Size of the input key, in bytes. Must be 16, 24, or 32.
 * @param[out] tag Generated output tag. Set to NULL to skip tag processing.
 * @param tagSize Input size of the tag to generate, in bytes. Must be 4, 6, 8, 10, 12, 14, or 16.
 * @return Status from encrypt operation
 */
status_t CAAM_AES_EncryptTagCcm(CAAM_Type *base,
                                caam_handle_t *handle,
                                const uint8_t *plaintext,
                                uint8_t *ciphertext,
                                uint32_t size,
                                const uint8_t *iv,
                                uint32_t ivSize,
                                const uint8_t *aad,
                                uint32_t aadSize,
                                const uint8_t *key,
                                uint32_t keySize,
                                uint8_t *tag,
                                uint32_t tagSize);

status_t CAAM_AES_EncryptTagCcmNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_desc_aes_ccm_t descriptor,
                                           const uint8_t *plaintext,
                                           uint8_t *ciphertext,
                                           uint32_t size,
                                           const uint8_t *iv,
                                           uint32_t ivSize,
                                           const uint8_t *aad,
                                           uint32_t aadSize,
                                           const uint8_t *key,
                                           uint32_t keySize,
                                           uint8_t *tag,
                                           uint32_t tagSize);

/*!
 * @brief Decrypts AES and authenticates using CCM block mode.
 *
 * Decrypts AES and optionally authenticates using CCM block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input cipher text to decrypt
 * @param[out] plaintext Output plain text.
 * @param size Size of input and output data in bytes. Zero means authentication data only.
 * @param iv Nonce
 * @param ivSize Length of the Nonce in bytes. Must be 7, 8, 9, 10, 11, 12, or 13.
 * @param aad Input additional authentication data. Can be NULL if aadSize is zero.
 * @param aadSize Input size in bytes of AAD. Zero means data mode only (authentication data skipped).
 * @param key Input key to use for decryption
 * @param keySize Size of the input key, in bytes. Must be 16, 24, or 32.
 * @param tag Received tag. Set to NULL to skip tag processing.
 * @param tagSize Input size of the received tag to compare with the computed tag, in bytes. Must be 4, 6, 8, 10, 12,
 *                14, or 16.
 * @return Status from decrypt operation
 */
status_t CAAM_AES_DecryptTagCcm(CAAM_Type *base,
                                caam_handle_t *handle,
                                const uint8_t *ciphertext,
                                uint8_t *plaintext,
                                uint32_t size,
                                const uint8_t *iv,
                                uint32_t ivSize,
                                const uint8_t *aad,
                                uint32_t aadSize,
                                const uint8_t *key,
                                uint32_t keySize,
                                const uint8_t *tag,
                                uint32_t tagSize);

status_t CAAM_AES_DecryptTagCcmNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_desc_aes_ccm_t descriptor,
                                           const uint8_t *ciphertext,
                                           uint8_t *plaintext,
                                           uint32_t size,
                                           const uint8_t *iv,
                                           uint32_t ivSize,
                                           const uint8_t *aad,
                                           uint32_t aadSize,
                                           const uint8_t *key,
                                           uint32_t keySize,
                                           const uint8_t *tag,
                                           uint32_t tagSize);

/*!
 * @brief Encrypts AES and tags using GCM block mode.
 *
 * Encrypts AES and optionally tags using GCM block mode. If plaintext is NULL, only the GHASH is calculated and output
 * in the 'tag' field.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plain text to encrypt
 * @param[out] ciphertext Output cipher text.
 * @param size Size of input and output data in bytes
 * @param iv Input initial vector
 * @param ivSize Size of the IV
 * @param aad Input additional authentication data
 * @param aadSize Input size in bytes of AAD
 * @param key Input key to use for encryption
 * @param keySize Size of the input key, in bytes. Must be 16, 24, or 32.
 * @param[out] tag Output hash tag. Set to NULL to skip tag processing.
 * @param tagSize Input size of the tag to generate, in bytes. Must be 4,8,12,13,14,15 or 16.
 * @return Status from encrypt operation
 */
status_t CAAM_AES_EncryptTagGcm(CAAM_Type *base,
                                caam_handle_t *handle,
                                const uint8_t *plaintext,
                                uint8_t *ciphertext,
                                uint32_t size,
                                const uint8_t *iv,
                                uint32_t ivSize,
                                const uint8_t *aad,
                                uint32_t aadSize,
                                const uint8_t *key,
                                uint32_t keySize,
                                uint8_t *tag,
                                uint32_t tagSize);

status_t CAAM_AES_EncryptTagGcmNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_desc_aes_gcm_t descriptor,
                                           const uint8_t *plaintext,
                                           uint8_t *ciphertext,
                                           uint32_t size,
                                           const uint8_t *iv,
                                           uint32_t ivSize,
                                           const uint8_t *aad,
                                           uint32_t aadSize,
                                           const uint8_t *key,
                                           uint32_t keySize,
                                           uint8_t *tag,
                                           uint32_t tagSize);

/*!
 * @brief Decrypts AES and authenticates using GCM block mode.
 *
 * Decrypts AES and optionally authenticates using GCM block mode. If ciphertext is NULL, only the GHASH is calculated
 * and compared with the received GHASH in 'tag' field.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input cipher text to decrypt
 * @param[out] plaintext Output plain text.
 * @param size Size of input and output data in bytes
 * @param iv Input initial vector
 * @param ivSize Size of the IV
 * @param aad Input additional authentication data
 * @param aadSize Input size in bytes of AAD
 * @param key Input key to use for encryption
 * @param keySize Size of the input key, in bytes. Must be 16, 24, or 32.
 * @param tag Input hash tag to compare. Set to NULL to skip tag processing.
 * @param tagSize Input size of the tag, in bytes. Must be 4, 8, 12, 13, 14, 15, or 16.
 * @return Status from decrypt operation
 */
status_t CAAM_AES_DecryptTagGcm(CAAM_Type *base,
                                caam_handle_t *handle,
                                const uint8_t *ciphertext,
                                uint8_t *plaintext,
                                uint32_t size,
                                const uint8_t *iv,
                                uint32_t ivSize,
                                const uint8_t *aad,
                                uint32_t aadSize,
                                const uint8_t *key,
                                uint32_t keySize,
                                const uint8_t *tag,
                                uint32_t tagSize);

status_t CAAM_AES_DecryptTagGcmNonBlocking(CAAM_Type *base,
                                           caam_desc_ges_ccm_t descriptor,
                                           caam_handle_t *handle,
                                           const uint8_t *ciphertext,
                                           uint8_t *plaintext,
                                           uint32_t size,
                                           const uint8_t *iv,
                                           uint32_t ivSize,
                                           const uint8_t *aad,
                                           uint32_t aadSize,
                                           const uint8_t *key,
                                           uint32_t keySize,
                                           const uint8_t *tag,
                                           uint32_t tagSize);

/*!
 *@}
 */ /* end of caam_driver_aes */

/*******************************************************************************
 * HASH API
 ******************************************************************************/

/*!
 * @addtogroup caam_driver_hash
 * @{
 */
/*!
 * @brief Initialize HASH context
 *
 * This function initializes the HASH.
 * Key shall be supplied if the underlaying algoritm is AES XCBC-MAC or CMAC.
 * Key shall be NULL if the underlaying algoritm is SHA.
 *
 * For XCBC-MAC, the key length must be 16. For CMAC, the key length can be
 * the AES key lengths supported by AES engine. For MDHA the key length argument
 * is ignored.
 *
 * This functions is used to initialize the context for both blocking and non-blocking
 * CAAM_HASH API.
 * For blocking CAAM HASH API, the HASH context contains all information required for context switch,
 * such as running hash or MAC. For non-blocking CAAM HASH API, the HASH context is used
 * to hold SGT. Therefore, the HASH context cannot be shared between blocking and non-blocking HASH API.
 * With one HASH context, either use only blocking HASH API or only non-blocking HASH API.
 *
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request.
 * @param[out] ctx Output hash context
 * @param algo Underlaying algorithm to use for hash computation.
 * @param key Input key (NULL if underlaying algorithm is SHA)
 * @param keySize Size of input key in bytes
 * @return Status of initialization
 */
status_t CAAM_HASH_Init(CAAM_Type *base,
                        caam_handle_t *handle,
                        caam_hash_ctx_t *ctx,
                        caam_hash_algo_t algo,
                        const uint8_t *key,
                        uint32_t keySize);

/*!
 * @brief Add data to current HASH
 *
 * Add data to current HASH. This can be called repeatedly with an arbitrary amount of data to be
 * hashed. The functions blocks. If it returns kStatus_Success, the running hash or mac
 * has been updated (CAAM has processed the input data), so the memory at input pointer
 * can be released back to system. The context is updated with the running hash or mac
 * and with all necessary information to support possible context switch.
 *
 * @param[in,out] ctx HASH context
 * @param input Input data
 * @param inputSize Size of input data in bytes
 * @return Status of the hash update operation
 */
status_t CAAM_HASH_Update(caam_hash_ctx_t *ctx, const uint8_t *input, uint32_t inputSize);

/*!
 * @brief Add input address and size to input data table
 *
 * Add data input pointer to a table maintained internally in the context.
 * Each call of this function creates one entry in the table.
 * The entry consists of the input pointer and inputSize.
 * All entries created by one or multiple calls of this function can be processed
 * in one call to CAAM_HASH_FinishNonBlocking() function.
 * Individual entries can point to non-continuous data in the memory.
 * The processing will occur in the order in which the CAAM_HASH_UpdateNonBlocking()
 * have been called.
 *
 * Memory pointers will be later accessed by CAAM (at time of CAAM_HASH_FinishNonBlocking()),
 * so the memory must stay valid
 * until CAAM_HASH_FinishNonBlocking() has been called and CAAM completes the processing.
 *
 * @param[in,out] ctx HASH context
 * @param input Input data
 * @param inputSize Size of input data in bytes
 * @return Status of the hash update operation
 */
status_t CAAM_HASH_UpdateNonBlocking(caam_hash_ctx_t *ctx, const uint8_t *input, uint32_t inputSize);

/*!
 * @brief Finalize hashing
 *
 * Outputs the final hash (computed by CAAM_HASH_Update()) and erases the context.
 *
 * @param[in,out] ctx Input hash context
 * @param[out] output Output hash data
 * @param[out] outputSize Output parameter storing the size of the output hash in bytes
 * @return Status of the hash finish operation
 */
status_t CAAM_HASH_Finish(caam_hash_ctx_t *ctx, uint8_t *output, uint32_t *outputSize);

/*!
 * @brief Finalize hashing
 *
 * The actual algorithm is computed with all input data, the memory pointers
 * are accessed by CAAM after the function returns.
 * The input data chunks have been specified by prior calls to CAAM_HASH_UpdateNonBlocking().
 * The function schedules the request at CAAM, then returns.
 * After a while, when the CAAM completes processing of the input data chunks,
 * the result is written to the output[] array, outputSize is written and the context
 * is cleared.
 *
 * @param[in,out] ctx Input hash context
 * @param[out] output Output hash data
 * @param[out] outputSize Output parameter storing the size of the output hash in bytes
 * @return Status of the hash finish operation
 */
status_t CAAM_HASH_FinishNonBlocking(caam_hash_ctx_t *ctx,
                                     caam_desc_hash_t descriptor,
                                     uint8_t *output,
                                     uint32_t *outputSize);

/*!
 * @brief Create HASH on given data
 *
 * Perform the full keyed XCBC-MAC/CMAC or SHA in one function call.
 *
 * Key shall be supplied if the underlaying algoritm is AES XCBC-MAC or CMAC.
 * Key shall be NULL if the underlaying algoritm is SHA.
 *
 * For XCBC-MAC, the key length must be 16. For CMAC, the key length can be
 * the AES key lengths supported by AES engine. For MDHA the key length argument
 * is ignored.
 *
 * The function is blocking.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request.
 * @param algo Underlaying algorithm to use for hash computation.
 * @param input Input data
 * @param inputSize Size of input data in bytes
 * @param key Input key (NULL if underlaying algorithm is SHA)
 * @param keySize Size of input key in bytes
 * @param[out] output Output hash data
 * @param[out] outputSize Output parameter storing the size of the output hash in bytes
 * @return Status of the one call hash operation.
 */
status_t CAAM_HASH(CAAM_Type *base,
                   caam_handle_t *handle,
                   caam_hash_algo_t algo,
                   const uint8_t *input,
                   uint32_t inputSize,
                   const uint8_t *key,
                   uint32_t keySize,
                   uint8_t *output,
                   uint32_t *outputSize);

/*!
 * @brief Create HASH on given data
 *
 * Perform the full keyed XCBC-MAC/CMAC or SHA in one function call.
 *
 * Key shall be supplied if the underlaying algoritm is AES XCBC-MAC or CMAC.
 * Key shall be NULL if the underlaying algoritm is SHA.
 *
 * For XCBC-MAC, the key length must be 16. For CMAC, the key length can be
 * the AES key lengths supported by AES engine. For MDHA the key length argument
 * is ignored.
 *
 * The function is non-blocking. The request is scheduled at CAAM.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request.
 * @param algo Underlaying algorithm to use for hash computation.
 * @param input Input data
 * @param inputSize Size of input data in bytes
 * @param key Input key (NULL if underlaying algorithm is SHA)
 * @param keySize Size of input key in bytes
 * @param[out] output Output hash data
 * @param[out] outputSize Output parameter storing the size of the output hash in bytes
 * @return Status of the one call hash operation.
 */
status_t CAAM_HASH_NonBlocking(CAAM_Type *base,
                               caam_handle_t *handle,
                               caam_desc_hash_t descriptor,
                               caam_hash_algo_t algo,
                               const uint8_t *input,
                               uint32_t inputSize,
                               const uint8_t *key,
                               uint32_t keySize,
                               uint8_t *output,
                               uint32_t *outputSize);
/*!
 *@}
 */ /* end of caam_driver_hash */

/*******************************************************************************
 * RNG API
 ******************************************************************************/

/*!
 * @addtogroup caam_driver_rng
 * @{
 */

/*!
 * @brief Initializes user configuration structure to default.
 *
 * This function initializes the configure structure to default value. the default
 * value are:
 * @code
 *     config->autoReseedInterval = 0;
 *     config->personalString = NULL;
 * @endcode
 *
 * @param config   User configuration structure.
 * @return status of the request
 */
status_t CAAM_RNG_GetDefaultConfig(caam_rng_config_t *config);

/*!
 * @brief Instantiate the CAAM RNG state handle
 *
 * This function instantiates CAAM RNG state handle.
 * The function is blocking and returns after CAAM has processed the request.
 *
 * @param base CAAM peripheral base address
 * @param handle CAAM jobRing used for this request
 * @param stateHandle RNG state handle to instantiate
 * @param config Pointer to configuration structure.
 * @return Status of the request
 */
status_t CAAM_RNG_Init(CAAM_Type *base,
                       caam_handle_t *handle,
                       caam_rng_state_handle_t stateHandle,
                       const caam_rng_config_t *config);

/*!
 * @brief Uninstantiate the CAAM RNG state handle
 *
 * This function uninstantiates CAAM RNG state handle.
 * The function is blocking and returns after CAAM has processed the request.
 *
 * @param base CAAM peripheral base address
 * @param handle jobRing used for this request.
 * @param stateHandle RNG state handle to uninstantiate
 * @return Status of the request
 */
status_t CAAM_RNG_Deinit(CAAM_Type *base, caam_handle_t *handle, caam_rng_state_handle_t stateHandle);

/*!
 * @brief Generate Secure Key
 *
 * This function generates random data writes it to Secure Key registers.
 * The function is blocking and returns after CAAM has processed the request.
 * RNG state handle 0 is always used.
 *
 * @param base CAAM peripheral base address
 * @param handle jobRing used for this request
 * @param additionalEntropy NULL or Pointer to optional 256-bit additional entropy.
 * @return Status of the request
 */
status_t CAAM_RNG_GenerateSecureKey(CAAM_Type *base, caam_handle_t *handle, caam_rng_generic256_t additionalEntropy);

/*!
 * @brief Reseed the CAAM RNG state handle
 *
 * This function reseeds the CAAM RNG state handle.
 * For a state handle in nondeterministic mode, the DRNG is seeded with 384 bits of
 * entropy from the TRNG and an optional 256-bit additional input from the descriptor
 * via the Class 1 Context Register.
 *
 * The function is blocking and returns after CAAM has processed the request.
 *
 * @param base CAAM peripheral base address
 * @param handle jobRing used for this request
 * @param stateHandle RNG state handle to reseed
 * @param additionalEntropy NULL or Pointer to optional 256-bit additional entropy.
 * @return Status of the request
 */
status_t CAAM_RNG_Reseed(CAAM_Type *base,
                         caam_handle_t *handle,
                         caam_rng_state_handle_t stateHandle,
                         caam_rng_generic256_t additionalEntropy);

/*!
 * @brief Get random data
 *
 * This function gets random data from CAAM RNG.
 *
 * The function is blocking and returns after CAAM has generated the requested data or an error occured.
 *
 * @param base CAAM peripheral base address
 * @param handle jobRing used for this request
 * @param stateHandle RNG state handle used to generate random data
 * @param[out] data Pointer address used to store random data
 * @param dataSize Size of the buffer pointed by the data parameter
 * @param dataType Type of random data to be generated
 * @param additionalEntropy NULL or Pointer to optional 256-bit additional entropy.
 * @return Status of the request
 */
status_t CAAM_RNG_GetRandomData(CAAM_Type *base,
                                caam_handle_t *handle,
                                caam_rng_state_handle_t stateHandle,
                                void *data,
                                size_t dataSize,
                                caam_rng_random_type_t dataType,
                                caam_rng_generic256_t additionalEntropy);

/*!
 * @brief Request random data
 *
 * This function schedules the request for random data from CAAM RNG.
 * Memory at memory pointers will be accessed by CAAM shortly after this function
 * returns, according to actual CAAM schedule.
 *
 * @param base CAAM peripheral base address
 * @param handle RNG handle used for this request
 * @param stateHandle RNG state handle used to generate random data
 * @param[out] descriptor memory for CAAM commands
 * @param[out] data Pointer address used to store random data
 * @param dataSize Size of the buffer pointed by the data parameter, in bytes.
 * @param dataType Type of random data to be generated.
 * @param additionalEntropy NULL or Pointer to optional 256-bit additional entropy.
 * @return status of the request
 */
status_t CAAM_RNG_GetRandomDataNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_rng_state_handle_t stateHandle,
                                           caam_desc_rng_t descriptor,
                                           void *data,
                                           size_t dataSize,
                                           caam_rng_random_type_t dataType,
                                           caam_rng_generic256_t additionalEntropy);

/*!
 *@}
 */ /* end of caam_driver_rng */

/*******************************************************************************
 * DES API
 ******************************************************************************/

/*!
 * @addtogroup caam_driver_des
 * @{
 */

/*!
 * @brief Encrypts DES using ECB block mode.
 *
 * Encrypts DES using ECB block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input and output data in bytes. Must be multiple of 8 bytes.
 * @param key Input key to use for encryption
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES_EncryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t key[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES_EncryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t key[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts DES using ECB block mode.
 *
 * Decrypts DES using ECB block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes. Must be multiple of 8 bytes.
 * @param key Input key to use for decryption
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES_DecryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t key[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES_DecryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t key[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts DES using CBC block mode.
 *
 * Encrypts DES using CBC block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Ouput ciphertext
 * @param size Size of input and output data in bytes
 * @param iv Input initial vector to combine with the first plaintext block.
 *           The iv does not need to be secret, but it must be unpredictable.
 * @param key Input key to use for encryption
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES_EncryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES_EncryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts DES using CBC block mode.
 *
 * Decrypts DES using CBC block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input data in bytes
 * @param iv Input initial vector to combine with the first plaintext block.
 *           The iv does not need to be secret, but it must be unpredictable.
 * @param key Input key to use for decryption
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES_DecryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES_DecryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts DES using CFB block mode.
 *
 * Encrypts DES using CFB block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param size Size of input data in bytes
 * @param iv Input initial block.
 * @param key Input key to use for encryption
 * @param[out] ciphertext Output ciphertext
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES_EncryptCfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES_EncryptCfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts DES using CFB block mode.
 *
 * Decrypts DES using CFB block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes
 * @param iv Input initial block.
 * @param key Input key to use for decryption
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES_DecryptCfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES_DecryptCfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts DES using OFB block mode.
 *
 * Encrypts DES using OFB block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input and output data in bytes
 * @param iv Input unique input vector. The OFB mode requires that the IV be unique
 *           for each execution of the mode under the given key.
 * @param key Input key to use for encryption
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES_EncryptOfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES_EncryptOfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts DES using OFB block mode.
 *
 * Decrypts DES using OFB block mode.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes. Must be multiple of 8 bytes.
 * @param iv Input unique input vector. The OFB mode requires that the IV be unique
 *           for each execution of the mode under the given key.
 * @param key Input key to use for decryption
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES_DecryptOfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES_DecryptOfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts triple DES using ECB block mode with two keys.
 *
 * Encrypts triple DES using ECB block mode with two keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input and output data in bytes. Must be multiple of 8 bytes.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES2_EncryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES2_EncryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts triple DES using ECB block mode with two keys.
 *
 * Decrypts triple DES using ECB block mode with two keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes. Must be multiple of 8 bytes.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES2_DecryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES2_DecryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts triple DES using CBC block mode with two keys.
 *
 * Encrypts triple DES using CBC block mode with two keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input and output data in bytes
 * @param iv Input initial vector to combine with the first plaintext block.
 *           The iv does not need to be secret, but it must be unpredictable.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES2_EncryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES2_EncryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts triple DES using CBC block mode with two keys.
 *
 * Decrypts triple DES using CBC block mode with two keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes
 * @param iv Input initial vector to combine with the first plaintext block.
 *           The iv does not need to be secret, but it must be unpredictable.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES2_DecryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES2_DecryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts triple DES using CFB block mode with two keys.
 *
 * Encrypts triple DES using CFB block mode with two keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input and output data in bytes
 * @param iv Input initial block.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES2_EncryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES2_EncryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts triple DES using CFB block mode with two keys.
 *
 * Decrypts triple DES using CFB block mode with two keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes
 * @param iv Input initial block.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES2_DecryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES2_DecryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts triple DES using OFB block mode with two keys.
 *
 * Encrypts triple DES using OFB block mode with two keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input and output data in bytes
 * @param iv Input unique input vector. The OFB mode requires that the IV be unique
 *           for each execution of the mode under the given key.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES2_EncryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES2_EncryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts triple DES using OFB block mode with two keys.
 *
 * Decrypts triple DES using OFB block mode with two keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes
 * @param iv Input unique input vector. The OFB mode requires that the IV be unique
 *           for each execution of the mode under the given key.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES2_DecryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES2_DecryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts triple DES using ECB block mode with three keys.
 *
 * Encrypts triple DES using ECB block mode with three keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input and output data in bytes. Must be multiple of 8 bytes.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @param key3 Third input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES3_EncryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES3_EncryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts triple DES using ECB block mode with three keys.
 *
 * Decrypts triple DES using ECB block mode with three keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes. Must be multiple of 8 bytes.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @param key3 Third input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES3_DecryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES3_DecryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts triple DES using CBC block mode with three keys.
 *
 * Encrypts triple DES using CBC block mode with three keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input data in bytes
 * @param iv Input initial vector to combine with the first plaintext block.
 *           The iv does not need to be secret, but it must be unpredictable.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @param key3 Third input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES3_EncryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES3_EncryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts triple DES using CBC block mode with three keys.
 *
 * Decrypts triple DES using CBC block mode with three keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes
 * @param iv Input initial vector to combine with the first plaintext block.
 *           The iv does not need to be secret, but it must be unpredictable.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @param key3 Third input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES3_DecryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES3_DecryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts triple DES using CFB block mode with three keys.
 *
 * Encrypts triple DES using CFB block mode with three keys.
 *
 * @param base CAAM peripheral base address
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input and ouput data in bytes
 * @param iv Input initial block.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @param key3 Third input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES3_EncryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES3_EncryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts triple DES using CFB block mode with three keys.
 *
 * Decrypts triple DES using CFB block mode with three keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input data in bytes
 * @param iv Input initial block.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @param key3 Third input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES3_DecryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES3_DecryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Encrypts triple DES using OFB block mode with three keys.
 *
 * Encrypts triple DES using OFB block mode with three keys.
 *
 * @param base CAAM peripheral base address
 * @param plaintext Input plaintext to encrypt
 * @param[out] ciphertext Output ciphertext
 * @param size Size of input and output data in bytes
 * @param iv Input unique input vector. The OFB mode requires that the IV be unique
 *           for each execution of the mode under the given key.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @param key3 Third input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES3_EncryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES3_EncryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE]);

/*!
 * @brief Decrypts triple DES using OFB block mode with three keys.
 *
 * Decrypts triple DES using OFB block mode with three keys.
 *
 * @param base CAAM peripheral base address
 * @param handle Handle used for this request. Specifies jobRing.
 * @param ciphertext Input ciphertext to decrypt
 * @param[out] plaintext Output plaintext
 * @param size Size of input and output data in bytes
 * @param iv Input unique input vector. The OFB mode requires that the IV be unique
 *           for each execution of the mode under the given key.
 * @param key1 First input key for key bundle
 * @param key2 Second input key for key bundle
 * @param key3 Third input key for key bundle
 * @return Status from encrypt/decrypt operation
 */
status_t CAAM_DES3_DecryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE]);

status_t CAAM_DES3_DecryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE]);

/*!
 *@}
 */ /* end of caam_driver_des */

/*!
 * @addtogroup caam_driver_pkha
 * @{
 */

/*!
 * @brief Converts from integer to Montgomery format.
 *
 * This function computes R2 mod N and optionally converts A or B into Montgomery format of A or B.
 *
 * @param base CAAM peripheral base address
 * @param N modulus
 * @param sizeN size of N in bytes
 * @param[in,out] A The first input in non-Montgomery format. Output Montgomery format of the first input.
 * @param[in,out] sizeA pointer to size variable. On input it holds size of input A in bytes. On output it holds size of
 *                Montgomery format of A in bytes.
 * @param[in,out] B Second input in non-Montgomery format. Output Montgomery format of the second input.
 * @param[in,out] sizeB pointer to size variable. On input it holds size of input B in bytes. On output it holds size of
 *                Montgomery format of B in bytes.
 * @param[out] R2 Output Montgomery factor R2 mod N.
 * @param[out] sizeR2 pointer to size variable. On output it holds size of Montgomery factor R2 mod N in bytes.
 * @param equalTime Run the function time equalized or no timing equalization.
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @return Operation status.
 */
status_t CAAM_PKHA_NormalToMontgomery(CAAM_Type *base,
                                      caam_handle_t *handle,
                                      const uint8_t *N,
                                      size_t sizeN,
                                      uint8_t *A,
                                      size_t *sizeA,
                                      uint8_t *B,
                                      size_t *sizeB,
                                      uint8_t *R2,
                                      size_t *sizeR2,
                                      caam_pkha_timing_t equalTime,
                                      caam_pkha_f2m_t arithType);

/*!
 * @brief Converts from Montgomery format to int.
 *
 * This function converts Montgomery format of A or B into int A or B.
 *
 * @param base CAAM peripheral base address
 * @param N modulus.
 * @param sizeN size of N modulus in bytes.
 * @param[in,out] A Input first number in Montgomery format. Output is non-Montgomery format.
 * @param[in,out] sizeA pointer to size variable. On input it holds size of the input A in bytes. On output it holds
 * size of non-Montgomery A in bytes.
 * @param[in,out] B Input first number in Montgomery format. Output is non-Montgomery format.
 * @param[in,out] sizeB pointer to size variable. On input it holds size of the input B in bytes. On output it holds
 * size of non-Montgomery B in bytes.
 * @param equalTime Run the function time equalized or no timing equalization.
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @return Operation status.
 */
status_t CAAM_PKHA_MontgomeryToNormal(CAAM_Type *base,
                                      caam_handle_t *handle,
                                      const uint8_t *N,
                                      size_t sizeN,
                                      uint8_t *A,
                                      size_t *sizeA,
                                      uint8_t *B,
                                      size_t *sizeB,
                                      caam_pkha_timing_t equalTime,
                                      caam_pkha_f2m_t arithType);

/*!
 * @brief Performs modular addition - (A + B) mod N.
 *
 * This function performs modular addition of (A + B) mod N, with either
 * integer or binary polynomial (F2m) inputs.  In the F2m form, this function is
 * equivalent to a bitwise XOR and it is functionally the same as subtraction.
 *
 * @param base CAAM peripheral base address
 * @param A first addend (integer or binary polynomial)
 * @param sizeA Size of A in bytes
 * @param B second addend (integer or binary polynomial)
 * @param sizeB Size of B in bytes
 * @param N modulus. For F2m operation this can be NULL, as N is ignored during F2m polynomial addition.
 * @param sizeN Size of N in bytes. This must be given for both integer and F2m polynomial additions.
 * @param[out] result Output array to store result of operation
 * @param[out] resultSize Output size of operation in bytes
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @return Operation status.
 */
status_t CAAM_PKHA_ModAdd(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *B,
                          size_t sizeB,
                          const uint8_t *N,
                          size_t sizeN,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType);

/*!
 * @brief Performs modular subtraction - (A - B) mod N.
 *
 * This function performs modular subtraction of (A - B) mod N with
 * integer inputs.
 *
 * @param base CAAM peripheral base address
 * @param A first addend (integer or binary polynomial)
 * @param sizeA Size of A in bytes
 * @param B second addend (integer or binary polynomial)
 * @param sizeB Size of B in bytes
 * @param N modulus
 * @param sizeN Size of N in bytes
 * @param[out] result Output array to store result of operation
 * @param[out] resultSize Output size of operation in bytes
 * @return Operation status.
 */
status_t CAAM_PKHA_ModSub1(CAAM_Type *base,
                           caam_handle_t *handle,
                           const uint8_t *A,
                           size_t sizeA,
                           const uint8_t *B,
                           size_t sizeB,
                           const uint8_t *N,
                           size_t sizeN,
                           uint8_t *result,
                           size_t *resultSize);

/*!
 * @brief Performs modular subtraction - (B - A) mod N.
 *
 * This function performs modular subtraction of (B - A) mod N,
 * with integer inputs.
 *
 * @param base CAAM peripheral base address
 * @param A first addend (integer or binary polynomial)
 * @param sizeA Size of A in bytes
 * @param B second addend (integer or binary polynomial)
 * @param sizeB Size of B in bytes
 * @param N modulus
 * @param sizeN Size of N in bytes
 * @param[out] result Output array to store result of operation
 * @param[out] resultSize Output size of operation in bytes
 * @return Operation status.
 */
status_t CAAM_PKHA_ModSub2(CAAM_Type *base,
                           caam_handle_t *handle,
                           const uint8_t *A,
                           size_t sizeA,
                           const uint8_t *B,
                           size_t sizeB,
                           const uint8_t *N,
                           size_t sizeN,
                           uint8_t *result,
                           size_t *resultSize);

/*!
 * @brief Performs modular multiplication - (A x B) mod N.
 *
 * This function performs modular multiplication with either integer or
 * binary polynomial (F2m) inputs.  It can optionally specify whether inputs
 * and/or outputs will be in Montgomery form or not.
 *
 * @param base CAAM peripheral base address
 * @param A first addend (integer or binary polynomial)
 * @param sizeA Size of A in bytes
 * @param B second addend (integer or binary polynomial)
 * @param sizeB Size of B in bytes
 * @param N modulus.
 * @param sizeN Size of N in bytes
 * @param[out] result Output array to store result of operation
 * @param[out] resultSize Output size of operation in bytes
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @param montIn Format of inputs
 * @param montOut Format of output
 * @param equalTime Run the function time equalized or no timing equalization. This argument is ignored for F2m modular
 * multiplication.
 * @return Operation status.
 */
status_t CAAM_PKHA_ModMul(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *B,
                          size_t sizeB,
                          const uint8_t *N,
                          size_t sizeN,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType,
                          caam_pkha_montgomery_form_t montIn,
                          caam_pkha_montgomery_form_t montOut,
                          caam_pkha_timing_t equalTime);

/*!
 * @brief Performs modular exponentiation - (A^E) mod N.
 *
 * This function performs modular exponentiation with either integer or
 * binary polynomial (F2m) inputs.
 *
 * @param base CAAM peripheral base address
 * @param A first addend (integer or binary polynomial)
 * @param sizeA Size of A in bytes
 * @param N modulus
 * @param sizeN Size of N in bytes
 * @param E exponent
 * @param sizeE Size of E in bytes
 * @param[out] result Output array to store result of operation
 * @param[out] resultSize Output size of operation in bytes
 * @param montIn Format of A input (normal or Montgomery)
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @param equalTime Run the function time equalized or no timing equalization.
 * @return Operation status.
 */
status_t CAAM_PKHA_ModExp(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *N,
                          size_t sizeN,
                          const uint8_t *E,
                          size_t sizeE,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType,
                          caam_pkha_montgomery_form_t montIn,
                          caam_pkha_timing_t equalTime);

/*!
 * @brief Performs modular reduction - (A) mod N.
 *
 * This function performs modular reduction with either integer or
 * binary polynomial (F2m) inputs.
 *
 * @param base CAAM peripheral base address
 * @param A first addend (integer or binary polynomial)
 * @param sizeA Size of A in bytes
 * @param N modulus
 * @param sizeN Size of N in bytes
 * @param[out] result Output array to store result of operation
 * @param[out] resultSize Output size of operation in bytes
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @return Operation status.
 */
status_t CAAM_PKHA_ModRed(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *N,
                          size_t sizeN,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType);

/*!
 * @brief Performs modular inversion - (A^-1) mod N.
 *
 * This function performs modular inversion with either integer or
 * binary polynomial (F2m) inputs.
 *
 * @param base CAAM peripheral base address
 * @param A first addend (integer or binary polynomial)
 * @param sizeA Size of A in bytes
 * @param N modulus
 * @param sizeN Size of N in bytes
 * @param[out] result Output array to store result of operation
 * @param[out] resultSize Output size of operation in bytes
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @return Operation status.
 */
status_t CAAM_PKHA_ModInv(CAAM_Type *base,
                          caam_handle_t *handle,
                          const uint8_t *A,
                          size_t sizeA,
                          const uint8_t *N,
                          size_t sizeN,
                          uint8_t *result,
                          size_t *resultSize,
                          caam_pkha_f2m_t arithType);

/*!
 * @brief Computes integer Montgomery factor R^2 mod N.
 *
 * This function computes a constant to assist in converting operands
 * into the Montgomery residue system representation.
 *
 * @param base CAAM peripheral base address
 * @param N modulus
 * @param sizeN Size of N in bytes
 * @param[out] result Output array to store result of operation
 * @param[out] resultSize Output size of operation in bytes
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @return Operation status.
 */
status_t CAAM_PKHA_ModR2(CAAM_Type *base,
                         caam_handle_t *handle,
                         const uint8_t *N,
                         size_t sizeN,
                         uint8_t *result,
                         size_t *resultSize,
                         caam_pkha_f2m_t arithType);

/*!
 * @brief Calculates the greatest common divisor - GCD (A, N).
 *
 * This function calculates the greatest common divisor of two inputs with
 * either integer or binary polynomial (F2m) inputs.
 *
 * @param base CAAM peripheral base address
 * @param A first value (must be smaller than or equal to N)
 * @param sizeA Size of A in bytes
 * @param N second value (must be non-zero)
 * @param sizeN Size of N in bytes
 * @param[out] result Output array to store result of operation
 * @param[out] resultSize Output size of operation in bytes
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @return Operation status.
 */
status_t CAAM_PKHA_GCD(CAAM_Type *base,
                       caam_handle_t *handle,
                       const uint8_t *A,
                       size_t sizeA,
                       const uint8_t *N,
                       size_t sizeN,
                       uint8_t *result,
                       size_t *resultSize,
                       caam_pkha_f2m_t arithType);

/*!
 * @brief Executes Miller-Rabin primality test.
 *
 * This function calculates whether or not a candidate prime number is likely
 * to be a prime.
 *
 * @param base CAAM peripheral base address
 * @param A initial random seed
 * @param sizeA Size of A in bytes
 * @param B number of trial runs
 * @param sizeB Size of B in bytes
 * @param N candidate prime integer
 * @param sizeN Size of N in bytes
 * @param[out] res True if the value is likely prime or false otherwise
 * @return Operation status.
 */
status_t CAAM_PKHA_PrimalityTest(CAAM_Type *base,
                                 caam_handle_t *handle,
                                 const uint8_t *A,
                                 size_t sizeA,
                                 const uint8_t *B,
                                 size_t sizeB,
                                 const uint8_t *N,
                                 size_t sizeN,
                                 bool *res);

/*!
 * @brief Adds elliptic curve points - A + B.
 *
 * This function performs ECC point addition over a prime field (Fp) or binary field (F2m) using
 * affine coordinates.
 *
 * @param base CAAM peripheral base address
 * @param A Left-hand point
 * @param B Right-hand point
 * @param N Prime modulus of the field
 * @param R2modN NULL (the function computes R2modN internally) or pointer to pre-computed R2modN (obtained from
 *               CAAM_PKHA_ModR2() function).
 * @param aCurveParam A parameter from curve equation
 * @param bCurveParam B parameter from curve equation (constant)
 * @param size Size in bytes of curve points and parameters
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @param[out] result Result point
 * @return Operation status.
 */
status_t CAAM_PKHA_ECC_PointAdd(CAAM_Type *base,
                                caam_handle_t *handle,
                                const caam_pkha_ecc_point_t *A,
                                const caam_pkha_ecc_point_t *B,
                                const uint8_t *N,
                                const uint8_t *R2modN,
                                const uint8_t *aCurveParam,
                                const uint8_t *bCurveParam,
                                size_t size,
                                caam_pkha_f2m_t arithType,
                                caam_pkha_ecc_point_t *result);

/*!
 * @brief Doubles elliptic curve points - B + B.
 *
 * This function performs ECC point doubling over a prime field (Fp) or binary field (F2m) using
 * affine coordinates.
 *
 * @param base CAAM peripheral base address
 * @param B Point to double
 * @param N Prime modulus of the field
 * @param aCurveParam A parameter from curve equation
 * @param bCurveParam B parameter from curve equation (constant)
 * @param size Size in bytes of curve points and parameters
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @param[out] result Result point
 * @return Operation status.
 */
status_t CAAM_PKHA_ECC_PointDouble(CAAM_Type *base,
                                   caam_handle_t *handle,
                                   const caam_pkha_ecc_point_t *B,
                                   const uint8_t *N,
                                   const uint8_t *aCurveParam,
                                   const uint8_t *bCurveParam,
                                   size_t size,
                                   caam_pkha_f2m_t arithType,
                                   caam_pkha_ecc_point_t *result);

/*!
 * @brief Multiplies an elliptic curve point by a scalar - E x (A0, A1).
 *
 * This function performs ECC point multiplication to multiply an ECC point by
 * a scalar integer multiplier over a prime field (Fp) or a binary field (F2m).
 *
 * @param base CAAM peripheral base address
 * @param A Point as multiplicand
 * @param E Scalar multiple
 * @param sizeE The size of E, in bytes
 * @param N Modulus, a prime number for the Fp field or Irreducible polynomial for F2m field.
 * @param R2modN NULL (the function computes R2modN internally) or pointer to pre-computed R2modN (obtained from
 *        CAAM_PKHA_ModR2() function).
 * @param aCurveParam A parameter from curve equation
 * @param bCurveParam B parameter from curve equation (C parameter for operation over F2m).
 * @param size Size in bytes of curve points and parameters
 * @param equalTime Run the function time equalized or no timing equalization.
 * @param arithType Type of arithmetic to perform (integer or F2m)
 * @param[out] result Result point
 * @param[out] infinity Output true if the result is point of infinity, and false otherwise. Writing of this output will
 * be ignored if the argument is NULL.
 * @return Operation status.
 */
status_t CAAM_PKHA_ECC_PointMul(CAAM_Type *base,
                                caam_handle_t *handle,
                                const caam_pkha_ecc_point_t *A,
                                const uint8_t *E,
                                size_t sizeE,
                                const uint8_t *N,
                                const uint8_t *R2modN,
                                const uint8_t *aCurveParam,
                                const uint8_t *bCurveParam,
                                size_t size,
                                caam_pkha_timing_t equalTime,
                                caam_pkha_f2m_t arithType,
                                caam_pkha_ecc_point_t *result,
                                bool *infinity);

/*!
 *@}
 */ /* end of caam_driver_pkha */

/*!
 *@}
 */ /* end of caam_driver */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_CAAM_H_ */
