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

#include "fsl_caam.h"

#define IRBAR0 (*(volatile uint32_t *)((uint8_t *)base + 0x1004u))
#define IRSR0 (*(volatile uint32_t *)((uint8_t *)base + 0x100Cu))
#define IRSAR0 (*(volatile uint32_t *)((uint8_t *)base + 0x1014u))
#define IRJAR0 (*(volatile uint32_t *)((uint8_t *)base + 0x101Cu))

#define ORBAR0 (*(volatile uint32_t *)((uint8_t *)base + 0x1024u))
#define ORSR0 (*(volatile uint32_t *)((uint8_t *)base + 0x102Cu))
#define ORJRR0 (*(volatile uint32_t *)((uint8_t *)base + 0x1034u))
#define ORSFR0 (*(volatile uint32_t *)((uint8_t *)base + 0x103Cu))

#define JRINTR0 (*(volatile uint32_t *)((uint8_t *)base + 0x104Cu))

#define BUILD_BUG(condition) ((void)sizeof(char[1 - 2 * !!(condition)]))
//#define BUILD_BUG_OR_ZERO(e) (sizeof(struct{ int:-!!(e);}))

static int32_t s_inJobRingIndex = 0;
static uint32_t s_inJobRing[4] = {0};
static uint32_t s_outJobRing[2 * ARRAY_SIZE(s_inJobRing)] = {0};

static const uint32_t templateAesEcb[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */
    /* 03 */ 0x22130000u, /* FIFO LOAD Message */
    /* 04 */ 0x00000000u, /* place: source address */
    /* 05 */ 0x60300000u, /* FIFO STORE Message */
    /* 06 */ 0x00000000u, /* place: destination address */
    /* 07 */ 0x82100200u, /* OPERATION: AES ECB Decrypt */
};

static const uint32_t templateAesCbc[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */
    /* 03 */ 0x12200010u, /* LOAD 16 bytes of iv to Class 1 Context Register */
    /* 04 */ 0x00000000u, /* place: iv address */
    /* 05 */ 0x22130000u, /* FIFO LOAD Message */
    /* 06 */ 0x00000000u, /* place: source address */
    /* 07 */ 0x60300000u, /* FIFO STORE Message */
    /* 08 */ 0x00000000u, /* place: destination address */
    /* 09 */ 0x82100100u, /* OPERATION: AES CBC Decrypt */
};

static const uint32_t templateAesCtr[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */
    /* 03 */ 0x12201010u, /* LOAD 16 bytes of CTR0 to Class 1 Context Register. Offset 16 bytes. */
    /* 04 */ 0x00000000u, /* place: CTR0 address */

    /* 05 */ 0x82100000u, /* OPERATION: AES CTR (de)crypt in Update mode */
    /* 06 */ 0x22130000u, /* FIFO LOAD Message */
    /* 07 */ 0x00000000u, /* place: source address */
    /* 08 */ 0x60300000u, /* FIFO STORE Message */
    /* 09 */ 0x00000000u, /* place: destination address */

    /* 10 */ 0xA2000001u, /* JMP always to next command. Done checkpoint (wait for Class 1 Done) */
    /* 11 */ 0x10880004u, /* LOAD Immediate to Clear Written Register. */
    /* 12 */ 0x08000004u, /* value for Clear Written Register: C1D and C1DS bits are set */
    /* 13 */ 0x22930010u, /* FIFO LOAD Message Immediate 16 bytes */
    /* 14 */ 0x00000000u, /* all zeroes 0-3 */

    /* 15 */ 0x00000000u, /* all zeroes 4-7 */
    /* 16 */ 0x00000000u, /* all zeroes 8-11 */
    /* 17 */ 0x00000000u, /* all zeroes 12-15 */
    /* 18 */ 0x60300010u, /* FIFO STORE Message 16 bytes */
    /* 19 */ 0x00000000u, /* place: counterlast[] block address */

    /* 20 */ 0x82100000u, /* OPERATION: AES CTR (de)crypt in Update mode */
    /* 21 */ 0x52201010u, /* STORE 16 bytes of CTRi from Class 1 Context Register offset 16 bytes. */
    /* 22 */ 0x00000000u, /* place: CTRi address */
};

static const uint32_t templateAesCcm[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */

    /* 03 */ 0x12A00010u, /* LOAD 16 immediate bytes of B0 to Class 1 Context Register. Offset 0 bytes. */
    /* 04 */ 0x00000000u, /* place: B0[0-3] */
    /* 05 */ 0x00000000u, /* place: B0[4-7] */
    /* 06 */ 0x00000000u, /* place: B0[8-11] */
    /* 07 */ 0x00000000u, /* place: B0[12-15] */

    /* 08 */ 0x12A01010u, /* LOAD 16 immediate bytes of CTR0 to Class 1 Context Register. Offset 16 bytes. */
    /* 09 */ 0x00000000u, /* place: CTR0[0-3] */
    /* 10 */ 0x00000000u, /* place: CTR0[4-7] */
    /* 11 */ 0x00000000u, /* place: CTR0[8-11] */
    /* 12 */ 0x00000000u, /* place: CTR0[12-15] */

    /* 13 */ 0x8210080Cu, /* OPERATION: AES CCM Decrypt Initialize/Finalize */

    /* 14 */ 0x22B00004u, /* FIFO LOAD additional authentication data. Immediate 32-bit word with aadSize encoded */
    /* 15 */ 0x00000000u, /* place: encoded aadSize followed by first byte(s) of authentication data */
    /* 16 */ 0x22310000u, /* FIFO LOAD additional authentication data. Flush as this is last data of AAD type. */
    /* 17 */ 0x00000000u, /* place: AAD address */

    /* 18 */ 0x22130000u, /* FIFO LOAD message */
    /* 19 */ 0x00000000u, /* place: message address */

    /* 20 */ 0x60300000u, /* FIFO STORE Message */
    /* 21 */ 0x00000000u, /* place: destination address */

    /* For encryption, write the computed and encrypted MAC to user buffer */
    /* 22 */ 0x52202000u, /* STORE from Class 1 context to tag */
    /* 23 */ 0x00000000u, /* place: tag address */

    /* For decryption, compare the computed tag with the received tag */

};

typedef enum _caam_algorithm
{
    kCAAM_AlgorithmAES = 0x10u << 16,
    kCAAM_AlgorithmDES = 0x20u << 16,
    kCAAM_Algorithm3DES = 0x21u << 16,
    kCAAM_AlgorithmSHA1 = 0x41u << 16,
    kCAAM_AlgorithmSHA224 = 0x42u << 16,
    kCAAM_AlgorithmSHA256 = 0x43u << 16,
    kCAAM_AlgorithmSHA384 = 0x44u << 16,
    kCAAM_AlgorithmSHA512 = 0x45u << 16,
} caam_algorithm_t;

typedef enum _caam_aai_symmetric_alg
{
    kCAAM_ModeCTR = 0x00U << 4,
    kCAAM_ModeCBC = 0x10U << 4,
    kCAAM_ModeECB = 0x20U << 4,
    kCAAM_ModeCFB = 0x30U << 4,
    kCAAM_ModeOFB = 0x40U << 4,
    kCAAM_ModeCMAC = 0x60U << 4,
    kCAAM_ModeXCBCMAC = 0x70U << 4,
    kCAAM_ModeCCM = 0x80U << 4,
    kCAAM_ModeGCM = 0x90U << 4,
} caam_aai_symmetric_alg_t;

typedef enum _caam_algorithm_state
{
    kCAAM_AlgStateUpdate = 0u,
    kCAAM_AlgStateInit = 1u,
    kCAAM_AlgStateFinal = 2u,
    kCAAM_AlgStateInitFinal = 3u,
} caam_algorithm_state_t;

/*******************************************************************************
 * HASH Definitions
 ******************************************************************************/
enum _caam_sha_digest_len
{
    kCAAM_RunLenSha1 = 28u,
    kCAAM_OutLenSha1 = 20u,
    kCAAM_RunLenSha224 = 40u,
    kCAAM_OutLenSha224 = 28u,
    kCAAM_RunLenSha256 = 40u,
    kCAAM_OutLenSha256 = 32u,
};

/*! Internal states of the HASH creation process */
typedef enum _caam_hash_algo_state
{
    kCAAM_HashInit = 1u, /*!< Key in the HASH context is the input key. */
    kCAAM_HashUpdate,    /*!< HASH context has algorithm specific context: MAC, K2 and K3 (XCBC-MAC), MAC and L (CMAC),
                             running digest (MDHA). Key in the HASH context is the derived key. */
} caam_hash_algo_state_t;

/*! 64-byte block represented as byte array or 16 32-bit words */
typedef union _caam_hash_block
{
    uint32_t w[CAAM_HASH_BLOCK_SIZE / 4]; /*!< array of 32-bit words */
    uint8_t b[CAAM_HASH_BLOCK_SIZE];      /*!< byte array */
} caam_hash_block_t;

/*! Definitions of indexes into hash context array */
typedef enum _caam_hash_ctx_indexes
{
    kCAAM_HashCtxKeyStartIdx = 12, /*!< context word array index where key is stored */
    kCAAM_HashCtxKeySize = 20,     /*!< context word array index where key size is stored */
    kCAAM_HashCtxNumWords = 21,    /*!< number of context array 32-bit words  */
} caam_hash_ctx_indexes;

typedef struct _caam_hash_ctx_internal
{
    caam_hash_block_t blk; /*!< memory buffer. only full 64-byte blocks are written to CAAM during hash updates */
    uint32_t word[kCAAM_HashCtxNumWords]; /*!< CAAM module context that needs to be saved/restored between CAAM jobs */
    uint32_t blksz;                       /*!< number of valid bytes in memory buffer */
    CAAM_Type *base;                      /*!< CAAM peripheral base address */
    caam_handle_t *handle;                /*!< CAAM handle (specifies jobRing and optional callback function) */
    caam_hash_algo_t algo;        /*!< selected algorithm from the set of supported algorithms in caam_hash_algo_t */
    caam_hash_algo_state_t state; /*!< finite machine state of the hash software process */
} caam_hash_ctx_internal_t;

/*! Definitions of indexes into hash job descriptor */
enum _caam_hash_sgt_index
{
    kCAAM_HashDescriptorSgtIdx = 13u, /*!< Index of the hash job descriptor[] where the two entry SGT starts. */
};

/*! One entry in the SGT */
typedef struct _caam_sgt_entry
{
    uint32_t zero;
    uint32_t address;
    uint32_t length;
    uint32_t offset;
} caam_sgt_entry_t;

/*! Definitions SGT entry type */
typedef enum _caam_hash_sgt_entry_type
{
    kCAAM_HashSgtEntryNotLast = 0u, /*!< Do not set the Final Bit in SGT entries */
    kCAAM_HashSgtEntryLast = 1u,    /*!< Sets Final Bit in the last SGT entry */
} caam_hash_sgt_entry_type_t;

/*! Two entry SGT, embedded in the hash job descriptor */
typedef caam_sgt_entry_t caam_hash_internal_sgt_t[2];

/*! Definitions of SGT type */
typedef enum _caam_hash_sgt_type
{
    kCAAM_HashSgtInternal = 0u, /*!< Two entry SGT is copied into the hash job descriptor. */
    kCAAM_HashSgtExternal = 1u, /*!< Use external SGT. */
} caam_hash_sgt_type_t;

enum _caam_hash_non_blocking_sgt_entries
{
    kCAAM_HashSgtMaxCtxEntries =
        (sizeof(caam_hash_block_t) + sizeof(uint32_t) * kCAAM_HashCtxKeyStartIdx) / sizeof(caam_sgt_entry_t),
};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * CAAM Common code static
 ******************************************************************************/
/*!
 * @brief Tests the correct key size.
 *
 * This function tests the correct key size.
 * @param keySize Input key length in bytes.
 * @return True if the key length is supported, false if not.
 */
bool caam_check_key_size(const uint32_t keySize)
{
    return ((keySize == 16u) || ((keySize == 24u)) || ((keySize == 32u)));
}

static inline status_t caam_in_job_ring_add(CAAM_Type *base, caam_job_ring_t jobRing, void *descaddr)
{
    /* adding new job to the s_inJobRing[] must be atomic
     * as this is global variable
     */
    uint32_t currPriMask = DisableGlobalIRQ();
    s_inJobRing[s_inJobRingIndex] = (uint32_t)descaddr;
    s_inJobRingIndex++;
    if (s_inJobRingIndex >= ARRAY_SIZE(s_inJobRing))
    {
        s_inJobRingIndex = 0;
    }
    IRJAR0 = 1;
    EnableGlobalIRQ(currPriMask);
    return kStatus_Success;
}

/* this function shall be only called inside CAAM driver critical section
 * because it accesses global variables.
 */
static inline status_t caam_out_job_ring_remove(CAAM_Type *base, caam_job_ring_t jobRing, int outIndex)
{
    s_outJobRing[outIndex++] = 0; /* clear descriptor address */
    s_outJobRing[outIndex] = 0;   /* clear status */

    ORJRR0 = 1;
    return 0;
}

static inline status_t caam_out_job_ring_test_and_remove(
    CAAM_Type *base, caam_job_ring_t jobRing, void *descriptor, bool *wait, bool *found)
{
    uint32_t currPriMask = DisableGlobalIRQ();
    int i;
    status_t status;

    *found = false;
    *wait = true;
    status = kStatus_Success;

    /* check if an interrupt or other thread consumed the result that we just saw */
    if (ORSFR0)
    {
        /* check if our descriptor is in the output job ring
         * look from the beginning of the out job ring
         */
        i = 0;

        while ((!*found) && (i < ARRAY_SIZE(s_outJobRing)))
        {
            if (s_outJobRing[i] == (uint32_t)descriptor)
            {
                *found = true;
                *wait = false;
                /* check for error in status word */
                if (s_outJobRing[i + 1])
                {
                    status = kStatus_Fail;
                }
                caam_out_job_ring_remove(base, jobRing, i);
            }
            else
            {
                /* try next result */
                i += 2u;
            }
        }
    }
    EnableGlobalIRQ(currPriMask);
    return status;
}

typedef union _caam_xcm_block_t
{
    uint32_t w[4]; /*!< CAAM context register is 16 bytes written as four 32-bit words */
    uint8_t b[16]; /*!< 16 octets block for CCM B0 and CTR0 and for GCM */
} caam_xcm_block_t;

static uint32_t swap_bytes(uint32_t in)
{
    return (((in & 0x000000ffu) << 24) | ((in & 0x0000ff00u) << 8) | ((in & 0x00ff0000u) >> 8) |
            ((in & 0xff000000u) >> 24));
}

static void caam_aes_ccm_context_init(
    uint32_t inputSize, const uint8_t *iv, uint32_t ivSize, uint32_t aadSize, uint32_t tagSize, void *b0, void *ctr0)
{
    caam_xcm_block_t blk;
    caam_xcm_block_t blkZero = {{0x0u, 0x0u, 0x0u, 0x0u}};

    int q; /* octet length of binary representation of the octet length of the payload. computed as (15 - n), where n is
              length of nonce(=ivSize) */
    uint8_t flags; /* flags field in B0 and CTR0 */

    /* compute B0 */
    memcpy(&blk, &blkZero, sizeof(blk));
    /* tagSize - size of output MAC */
    q = 15 - ivSize;
    flags = (uint8_t)(8 * ((tagSize - 2) / 2) + q - 1); /* 8*M' + L' */
    if (aadSize)
    {
        flags |= 0x40; /* Adata */
    }
    blk.b[0] = flags;                 /* flags field */
    blk.w[3] = swap_bytes(inputSize); /* message size, most significant byte first */
    memcpy(&blk.b[1], iv, ivSize);    /* nonce field */

    /* Write B0 data to the context register.
     */
    memcpy(b0, &blk.b[0], 16);

    /* Write CTR0 to the context register.
     */
    memcpy(&blk, &blkZero, sizeof(blk)); /* ctr(0) field = zero */
    blk.b[0] = q - 1;                    /* flags field */
    memcpy(&blk.b[1], iv, ivSize);       /* nonce field */
    memcpy(ctr0, &blk.b[0], 16);
}

status_t caam_aes_ccm_non_blocking(CAAM_Type *base,
                                   caam_handle_t *handle,
                                   caam_desc_aes_ccm_t descriptor,
                                   const uint8_t *input,
                                   uint8_t *output,
                                   uint32_t size,
                                   const uint8_t *iv,
                                   uint32_t ivSize,
                                   const uint8_t *aad,
                                   uint32_t aadSize,
                                   const uint8_t *key,
                                   uint32_t keySize,
                                   uint32_t tag,
                                   uint32_t tagSize,
                                   int encrypt)
{
    BUILD_BUG(sizeof(templateAesCcm) != sizeof(caam_desc_aes_ccm_t));

    /* get template descriptor and it's size */
    uint32_t descriptorSize = ARRAY_SIZE(templateAesCcm);
    memcpy(descriptor, templateAesCcm, sizeof(templateAesCcm));

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* key address and key size */
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;

    /* B0 and CTR0 */
    caam_aes_ccm_context_init(size, iv, ivSize, aadSize, tagSize, &descriptor[4], &descriptor[9]);

    /* Encrypt decrypt */
    if (encrypt)
    {
        descriptor[13] |= 1u; /* ENC */
    }
    else if (tag)
    {
        descriptor[13] |= 2u; /* ICV_TEST */
    }
    else
    {
        /* decrypt with tag NULL (skip tag processing). nothing needs to be changed in descriptor[13] for this case */
    }

    /* AAD address and size */
    /* encoding is two octets, msbyte first */
    if (aadSize)
    {
        uint32_t swapped = swap_bytes(aadSize);
        uint32_t sz;
        memcpy(&descriptor[15], ((uint8_t *)&swapped) + sizeof(uint16_t), sizeof(uint16_t));
        sz = aadSize > 2u ? 2u : aadSize;                  /* limit aad to the end of 16 bytes blk */
        memcpy(((uint8_t *)&descriptor[15]) + 2, aad, sz); /* fill B1 with aad */
        /* track consumed AAD. sz bytes have been moved to fifo. */
        aadSize -= sz;
        aad += sz;

        if (!aadSize)
        {
            /* in case aadSize is 1 or 2, we add Flush bit to the command and skip FIFO LOAD AAD */
            descriptor[14] |= 0x00010000; /* Flush (last AAD data) */
            descriptor[16] = 0xA0000002u; /* jump to current index + 2 (=18) */
        }
        else
        {
            descriptor[16] |= (aadSize & 0x0000FFFFu);
            descriptor[17] = (uint32_t)aad;
        }
    }
    else
    {
        /* no AAD, jump directly to message */
        descriptor[14] = 0xA0000004u; /* jump to current index + 4 (=18) */
    }

    /* Message source address and size */
    descriptor[18] |= (size & 0x0000FFFFu);
    descriptor[19] = (uint32_t)input;

    /* Message destination address and size */
    descriptor[20] |= (size & 0x0000FFFFu);
    descriptor[21] = (uint32_t)output;

    if (tag)
    {
        if (encrypt)
        {
            descriptor[22] |= (tagSize & 0xFFu);
        }
        else
        {
            descriptor[22] = (0x223B0000u | (tagSize & 0xFFu));
        }
        descriptor[23] = (uint32_t)tag;
    }
    else
    {
        /* tag is NULL, skip tag processing */
        descriptor[22] = 0xA0C00000u; /* always halt with status 0x0 (normal) */
    }

    /* add operation specified by descriptor to CAAM Job Ring */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

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
                                           uint32_t tagSize)
{
    return caam_aes_ccm_non_blocking(base, handle, descriptor, plaintext, ciphertext, size, iv, ivSize, aad, aadSize,
                                     key, keySize, (uint32_t)tag, tagSize, 1);
}

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
                                           uint32_t tagSize)
{
    return caam_aes_ccm_non_blocking(base, handle, descriptor, ciphertext, plaintext, size, iv, ivSize, aad, aadSize,
                                     key, keySize, (uint32_t)tag, tagSize, 0);
}

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
                                      uint32_t *szLeft)
{
    BUILD_BUG(sizeof(templateAesCtr) != sizeof(caam_desc_aes_ctr_t));

    /* get template descriptor and it's size */
    uint32_t descriptorSize = ARRAY_SIZE(templateAesCtr);
    memcpy(descriptor, templateAesCtr, sizeof(templateAesCtr));

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* key address and key size */
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;

    /* descriptor[3] configures 16 bytes length for CTR0 in templateAesCtr */
    descriptor[4] = (uint32_t)counter;

    /* source address and size */
    descriptor[6] |= (size & 0x0000FFFFu);
    descriptor[7] = (uint32_t)input;

    /* destination address and size */
    descriptor[8] |= (size & 0x0000FFFFu);
    descriptor[9] = (uint32_t)output;

    /* AES CTR Crypt OPERATION in descriptor[5]
     * Algorithm State (AS) in template is Update (0h)
     * Only in case we are chaining the AES CTR calls (counterlast[] != NULL),
     * we have to change the algorithm state to Finalize (2h)
     * and so the CTRi for the last message block is not written to Class 1 Context.
     * This allows us to repeat AES CTR of the last CTRi, with destination to counterlast[],
     * and with using all zeroes in message data, the counterlast[] gets ECB of the last CTRi.
     */

    /* if counterlast or szLeft is NULL, the caller is not interested in AES of last counter
     * Thus, we can skip the counterlast processing
     * and only read the last CTRi from context.
     * So, we replace descriptor[10] with a jump command to STORE
     */
    if ((counterlast == NULL) || (szLeft == NULL))
    {
        /*  To create an unconditional jump, use TEST TYPE = 00 (all specified conditions true) and
            clear all TEST CONDITION bits because the tested condition is considered to be true if
            no test condition bits are set. */
        descriptor[10] = 0xA000000Bu; /* jump to current index + 11 (=21) */
    }
    else
    {
        uint32_t lastSize;

        descriptor[5] |= 0x08u; /* finalize */
        descriptor[19] = (uint32_t)counterlast;

        lastSize = size % 16u;
        if (lastSize)
        {
            *szLeft = 16u - lastSize;
        }
        else
        {
            *szLeft = 0;
            /* descriptor[10] = 0xA000000Bu; */ /* jump to current index + 11 (=21) */
        }
    }

    /* read last CTRi from AES back to caller */
    descriptor[22] = (uint32_t)counter;

    /* add operation specified by descriptor to CAAM Job Ring */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_AES_EncryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_ecb_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t *key,
                                        uint32_t keySize)
{
    BUILD_BUG(sizeof(templateAesEcb) != sizeof(caam_desc_aes_ecb_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateAesEcb);

    memcpy(descriptor, templateAesEcb, sizeof(templateAesEcb));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;
    descriptor[3] |= (size & 0x0000FFFFu);
    descriptor[4] = (uint32_t)plaintext;
    descriptor[5] |= (size & 0x0000FFFFu);
    descriptor[6] = (uint32_t)ciphertext;
    descriptor[7] |= 1u; /* add ENC bit to specify Encrypt OPERATION */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_AES_DecryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_ecb_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t *key,
                                        uint32_t keySize)
{
    BUILD_BUG(sizeof(templateAesEcb) != sizeof(caam_desc_aes_ecb_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateAesEcb);

    memcpy(descriptor, templateAesEcb, sizeof(templateAesEcb));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;
    descriptor[3] |= (size & 0x0000FFFFu);
    descriptor[4] = (uint32_t)ciphertext;
    descriptor[5] |= (size & 0x0000FFFFu);
    descriptor[6] = (uint32_t)plaintext;

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_AES_EncryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_cbc_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t *iv,
                                        const uint8_t *key,
                                        uint32_t keySize)
{
    BUILD_BUG(sizeof(templateAesCbc) != sizeof(caam_desc_aes_cbc_t));

    /* get template descriptor and it's size */
    uint32_t descriptorSize = ARRAY_SIZE(templateAesCbc);
    memcpy(descriptor, templateAesCbc, sizeof(templateAesCbc));

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* key address and key size */
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;

    /* descriptor[3] configures 16 bytes length for IV in templateAesCbc */
    descriptor[4] = (uint32_t)iv;

    /* source address and size */
    descriptor[5] |= (size & 0x0000FFFFu);
    descriptor[6] = (uint32_t)plaintext;

    /* destination address and size */
    descriptor[7] |= (size & 0x0000FFFFu);
    descriptor[8] = (uint32_t)ciphertext;

    /* AES CBC */
    descriptor[9] |= 1u; /* add ENC bit to specify Encrypt OPERATION */

    /* add operation specified by descriptor to CAAM Job Ring */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_AES_DecryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_aes_cbc_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t *iv,
                                        const uint8_t *key,
                                        uint32_t keySize)
{
    BUILD_BUG(sizeof(templateAesCbc) != sizeof(caam_desc_aes_cbc_t));

    /* get template descriptor and it's size */
    uint32_t descriptorSize = ARRAY_SIZE(templateAesCbc);
    memcpy(descriptor, templateAesCbc, sizeof(templateAesCbc));

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* key address and key size */
    descriptor[1] |= (keySize & 0x3FFu);
    descriptor[2] = (uint32_t)key;

    /* descriptor[3] configures 16 bytes length for IV in templateAesCbc */
    descriptor[4] = (uint32_t)iv;

    /* source address and size */
    descriptor[5] |= (size & 0x0000FFFFu);
    descriptor[6] = (uint32_t)ciphertext;

    /* destination address and size */
    descriptor[7] |= (size & 0x0000FFFFu);
    descriptor[8] = (uint32_t)plaintext;

    /* AES CBC Decrypt OPERATION in descriptor[9] */

    /* add operation specified by descriptor to CAAM Job Ring */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

void CAAM_Init(CAAM_Type *base)
{
    CAAM_MCR_REG = 0x80000000; /* reset */
    CAAM_MCR_REG = 0x90000000; /* reset DMA */
    CAAM_MCR_REG = 0;
    *(volatile uint32_t *)0x4001701C = 0x00FF00FF;
    *(volatile uint32_t *)0x40017020 = 0x00FF00FF;

    /* Job Ring 0 Configuration
     * number of entries in both input and output ring is equal.
     * Note the size of an entry is different. an entry in the input ring is a 32-bit word.
     * an entry in the output ring is two 32-bit words. (descriptor pointer followed by termination status word)
     */
    IRBAR0 = (uint32_t)s_inJobRing;
    IRSR0 = ARRAY_SIZE(s_inJobRing);
    ORBAR0 = (uint32_t)s_outJobRing;
    ORSR0 = ARRAY_SIZE(s_outJobRing) / 2;

    /*
     * Instantiate RNG in normal (non-deterministic) mode and load the JDKEK, TDKEK and TDSK registers
     * this step is required for example
     * for FIFO STORE command to be able to store Key register as Black key
     * for example during AES XCBC-MAC context switch (need to store derived key K1 to memory)
     */
    caam_handle_t handle;
    handle.jobRing = kCAAM_JobRing0;

    caam_rng_config_t rngConfig;
    CAAM_RNG_GetDefaultConfig(&rngConfig);

    CAAM_RNG_Init(base, &handle, kCAAM_RngStateHandle0, &rngConfig);
    CAAM_RNG_GenerateSecureKey(base, &handle, NULL);
}

status_t CAAM_Wait(CAAM_Type *base, caam_handle_t *handle, void *descriptor, caam_wait_mode_t mode)
{
    /* poll output ring for the specified job descriptor */
    status_t status;
    bool wait;
    bool found;

    wait = true;
    status = kStatus_Success;
    found = false;

    while (wait)
    {
        /* any result available on this job ring? */
        if (ORSFR0)
        {
            status = caam_out_job_ring_test_and_remove(base, handle->jobRing, descriptor, &wait, &found);
        }

        /* non-blocking mode polls output ring once */
        if (mode == kCAAM_Nonblocking)
        {
            wait = false; /* exit the while() */
            if (!found)
            {
                status = kStatus_Again; /* job not in the tested ring */
            }
        }
    }
    return status;
}

status_t CAAM_AES_EncryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t *key,
                             uint32_t keySize)
{
    caam_desc_aes_ecb_t descBuf;
    status_t status;

    status = CAAM_AES_EncryptEcbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, key, keySize);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_DecryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t *key,
                             uint32_t keySize)
{
    caam_desc_aes_ecb_t descBuf;
    status_t status;

    status = CAAM_AES_DecryptEcbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, key, keySize);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_EncryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t iv[16],
                             const uint8_t *key,
                             uint32_t keySize)
{
    caam_desc_aes_cbc_t descBuf;
    status_t status;

    status = CAAM_AES_EncryptCbcNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key, keySize);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_DecryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t iv[16],
                             const uint8_t *key,
                             uint32_t keySize)
{
    caam_desc_aes_cbc_t descBuf;
    status_t status;

    status = CAAM_AES_DecryptCbcNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key, keySize);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_AES_CryptCtr(CAAM_Type *base,
                           caam_handle_t *handle,
                           const uint8_t *input,
                           uint8_t *output,
                           uint32_t size,
                           uint8_t counter[16],
                           const uint8_t *key,
                           uint32_t keySize,
                           uint8_t counterlast[16],
                           uint32_t *szLeft)
{
    caam_desc_aes_ctr_t descBuf;
    status_t status;

    status = CAAM_AES_CryptCtrNonBlocking(base, handle, descBuf, input, output, size, counter, key, keySize,
                                          counterlast, szLeft);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

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
                                uint32_t tagSize)
{
    caam_desc_aes_ccm_t descBuf;
    status_t status;

    status = CAAM_AES_EncryptTagCcmNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, ivSize, aad,
                                               aadSize, key, keySize, tag, tagSize);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

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
                                uint32_t tagSize)
{
    caam_desc_aes_ccm_t descBuf;
    status_t status;

    status = CAAM_AES_DecryptTagCcmNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, ivSize, aad,
                                               aadSize, key, keySize, tag, tagSize);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

/*******************************************************************************
 * HASH Code static
 ******************************************************************************/
static status_t caam_hash_check_input_alg(caam_hash_algo_t algo)
{
    if ((algo != kCAAM_XcbcMac) && (algo != kCAAM_Cmac) && (algo != kCAAM_Sha1) && (algo != kCAAM_Sha224) &&
        (algo != kCAAM_Sha256))
    {
        return kStatus_InvalidArgument;
    }
    return kStatus_Success;
}

static inline bool caam_hash_alg_is_cmac(caam_hash_algo_t algo)
{
    return ((algo == kCAAM_XcbcMac) || (algo == kCAAM_Cmac));
}

static inline bool caam_hash_alg_is_sha(caam_hash_algo_t algo)
{
    return ((algo == kCAAM_Sha1) || (algo == kCAAM_Sha224) || (algo == kCAAM_Sha256));
}

static status_t caam_hash_check_input_args(
    CAAM_Type *base, caam_hash_ctx_t *ctx, caam_hash_algo_t algo, const uint8_t *key, uint32_t keySize)
{
    /* Check validity of input algorithm */
    if (kStatus_Success != caam_hash_check_input_alg(algo))
    {
        return kStatus_InvalidArgument;
    }

    if ((NULL == ctx) || (NULL == base))
    {
        return kStatus_InvalidArgument;
    }

    if (caam_hash_alg_is_cmac(algo))
    {
        if ((NULL == key) || (!caam_check_key_size(keySize)))
        {
            return kStatus_InvalidArgument;
        }
    }

    return kStatus_Success;
}

static status_t caam_hash_check_context(caam_hash_ctx_internal_t *ctxInternal, const uint8_t *data)
{
    if ((NULL == data) || (NULL == ctxInternal) || (NULL == ctxInternal->base) ||
        (kStatus_Success != caam_hash_check_input_alg(ctxInternal->algo)))
    {
        return kStatus_InvalidArgument;
    }
    return kStatus_Success;
}

static uint32_t caam_hash_algo2mode(caam_hash_algo_t algo, uint32_t algState, uint32_t *algOutSize)
{
    uint32_t modeReg = 0u;
    uint32_t outSize = 0u;

    /* Set CAAM algorithm */
    switch (algo)
    {
        case kCAAM_XcbcMac:
            modeReg = (uint32_t)kCAAM_AlgorithmAES | (uint32_t)kCAAM_ModeXCBCMAC;
            outSize = 16u;
            break;
        case kCAAM_Cmac:
            modeReg = (uint32_t)kCAAM_AlgorithmAES | (uint32_t)kCAAM_ModeCMAC;
            outSize = 16u;
            break;
        case kCAAM_Sha1:
            modeReg = (uint32_t)kCAAM_AlgorithmSHA1;
            outSize = kCAAM_OutLenSha1;
            break;
        case kCAAM_Sha224:
            modeReg = (uint32_t)kCAAM_AlgorithmSHA224;
            outSize = kCAAM_OutLenSha224;
            break;
        case kCAAM_Sha256:
            modeReg = (uint32_t)kCAAM_AlgorithmSHA256;
            outSize = kCAAM_OutLenSha256;
            break;
        default:
            break;
    }

    modeReg |= algState;
    if (algOutSize)
    {
        *algOutSize = outSize;
    }

    return modeReg;
}

static uint32_t caam_hash_algo2ctx_size(caam_hash_algo_t algo, uint32_t how)
{
    uint32_t ctxSize = 0u;

    /* Size of context in bytes for context switching */
    switch (algo)
    {
        case kCAAM_XcbcMac:
            if (how == 0)
            {
                ctxSize = 48u; /* add K3 and K2 */
            }
            else
            {
                ctxSize = 16u; /* only running or final MAC during UPDATE or FINALIZE or INITIALIZE/FINALIZE */
            }
            break;
        case kCAAM_Cmac:
            if (how == 0)
            {
                ctxSize = 32u; /* add L */
            }
            else
            {
                ctxSize = 16u; /* only running or final MAC during UPDATE or FINALIZE or INITIALIZE/FINALIZE */
            }
            break;
        case kCAAM_Sha1:
            ctxSize = 28u;
            break;
        case kCAAM_Sha224:
            ctxSize = 40u;
            break;
        case kCAAM_Sha256:
            ctxSize = 40u;
            break;
        default:
            break;
    }
    return ctxSize;
}

status_t CAAM_HASH_Init(CAAM_Type *base,
                        caam_handle_t *handle,
                        caam_hash_ctx_t *ctx,
                        caam_hash_algo_t algo,
                        const uint8_t *key,
                        uint32_t keySize)
{
    status_t ret;
    caam_hash_ctx_internal_t *ctxInternal;
    uint32_t i;

    ret = caam_hash_check_input_args(base, ctx, algo, key, keySize);
    if (ret != kStatus_Success)
    {
        return ret;
    }

    /* set algorithm in context struct for later use */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    ctxInternal->algo = algo;
    for (i = 0; i < kCAAM_HashCtxNumWords; i++)
    {
        ctxInternal->word[i] = 0u;
    }

    /* Steps required only using AES engine */
    if (caam_hash_alg_is_cmac(algo))
    {
        /* store input key and key length in context struct for later use */
        ctxInternal->word[kCAAM_HashCtxKeySize] = keySize;
        memcpy(&ctxInternal->word[kCAAM_HashCtxKeyStartIdx], key, keySize);
    }

    ctxInternal->blksz = 0u;
    for (i = 0; i < ARRAY_SIZE(ctxInternal->blk.w); i++)
    {
        ctxInternal->blk.w[0] = 0u;
    }
    ctxInternal->state = kCAAM_HashInit;
    ctxInternal->base = base;
    ctxInternal->handle = handle;

    return kStatus_Success;
}

static const uint32_t templateHash[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x00000000u, /* KEY */
    /* 02 */ 0x00000000u, /* place: key address */
    /* 03 */ 0x10200000u, /* LOAD bytes to Class Context Register. Offset 0 bytes. */
    /* 04 */ 0x00000000u, /* place: context address */

    /* 05 */ 0x80000000u, /* OPERATION (place either AES MAC or MDHA SHA) */
    /* 06 */ 0x21170000u, /* FIFO LOAD Class Message via SGT */
    /* 07 */ 0x00000000u, /* place: SGT address */
    /* 08 */ 0x50200000u, /* STORE bytes from Class Context Register offset 0 bytes. */
    /* 09 */ 0x00000000u, /* place: context address */

    /* 10 */ 0x60240000u, /* FIFO STORE from KEY to memory. */
    /* 11 */ 0x00000000u, /* place: derived key address ECB encrypted */
    /* 12 */ 0xA0C00000u, /* halt always with status 0 */

    /* 13 */ 0x00000000u, /* SGT entry 0 word 0 */
    /* 14 */ 0x00000000u, /* SGT entry 0 word 1 */
    /* 15 */ 0x00000000u, /* SGT entry 0 word 2 */
    /* 16 */ 0x00000000u, /* SGT entry 0 word 3 */

    /* 17 */ 0x00000000u, /* SGT entry 1 word 0 */
    /* 18 */ 0x00000000u, /* SGT entry 1 word 1 */
    /* 19 */ 0x00000000u, /* SGT entry 1 word 2 */
    /* 20 */ 0x00000000u, /* SGT entry 1 word 3 */
};

/*!
 * @brief Add data chunk to SGT table. Append after uncomplete block in ctxInternal if there is any.
 *
 * @param ctxInternal uncomplete block in the hash context - to be inserted before new data chunk
 * @param input new data chunk to insert
 * @param inputSize size in bytes of new data chunk to insert
 * @param numRemain number of bytes that remain in the last uncomplete block
 * @param algState in FINALIZE or INITIALIZE/FINALIZE we add also last uncomplete block bytes
 * @param sgt address of the SGT
 * @param last last call to this function adds Final Bit
 */
static uint32_t caam_hash_sgt_insert(caam_hash_ctx_internal_t *ctxInternal,
                                     const uint8_t *input,
                                     uint32_t inputSize,
                                     uint32_t *numRemain,
                                     caam_algorithm_state_t algState,
                                     caam_sgt_entry_t *sgt,
                                     caam_hash_sgt_entry_type_t last)
{
    /* configure SGT
     * *64 bytes multiple in kCAAM_HashInit or kCAAM_HashUpdate
     * *arbitrary amount of data in kCAAM_HashInitFinal or kCAAM_HashFinal
     * min 1 and max 2 SGT entries
     * 1) if there is any data in the context buffer, use it as one entry
     * 2) input as one entry
     */
    uint32_t numBlocks;
    uint32_t remain;
    uint32_t num;
    uint32_t currSgtEntry;

    uint32_t ctxBlksz = ctxInternal ? ctxInternal->blksz : 0;
    uint32_t ctxBlkAddr = ctxInternal ? (uint32_t)&ctxInternal->blk.b[0] : 0;

    currSgtEntry = 0;
    numBlocks = (inputSize + ctxBlksz) / CAAM_HASH_BLOCK_SIZE;
    remain = (inputSize + ctxBlksz) % CAAM_HASH_BLOCK_SIZE;

    /* number of bytes for processing
     * only full block multiple in INITIALIZE or UPDATE
     * any size in INITIALIZE/FINALIZE or FINALIZE
     */
    num = (CAAM_HASH_BLOCK_SIZE * numBlocks);
    if ((algState == kCAAM_AlgStateInitFinal) || (algState == kCAAM_AlgStateFinal))
    {
        num += remain; /* add also uncomplete bytes from last block in one of FINALIZE states */
        remain = 0;
    }
    if (numRemain)
    {
        *numRemain = remain;
    }

    if (ctxBlksz || (0 == ctxBlksz + inputSize))
    {
        sgt[currSgtEntry].address = ctxBlkAddr;
        sgt[currSgtEntry].length = ctxBlksz;
        if ((kCAAM_HashSgtEntryLast == last) && (!inputSize))
        {
            sgt[currSgtEntry].length |= 0x40000000u; /* Final SG entry */
        }
        currSgtEntry++;
    }

    if (inputSize)
    {
        /* number of bytes for processing
         * only full block multiple in INITIALIZE or UPDATE
         * any size in INITIALIZE/FINALIZE or FINALIZE
         */
        sgt[currSgtEntry].address = (uint32_t)input;
        sgt[currSgtEntry].length = inputSize - remain;
        if (kCAAM_HashSgtEntryLast == last)
        {
            sgt[currSgtEntry].length |= 0x40000000u; /* Final SG entry */
        }
    }
    return num; /* no of bytes processed in total by these 1 or 2 SGT entries */
}

/*!
 * @brief Create job descriptor for the HASH request and schedule at CAAM job ring
 *
 *
 */
static status_t caam_hash_schedule_input_data(CAAM_Type *base,
                                              caam_handle_t *handle,
                                              caam_hash_algo_t algo,
                                              caam_sgt_entry_t *sgt,
                                              uint32_t dataSize,
                                              caam_hash_sgt_type_t sgtType,
                                              caam_algorithm_state_t algState,
                                              caam_desc_hash_t descriptor,
                                              uint32_t *outputSize,
                                              void *output,
                                              void *context,
                                              uint32_t keyAddr,
                                              uint32_t keySize)
{
    BUILD_BUG(sizeof(templateHash) != sizeof(caam_desc_hash_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateHash);
    uint32_t algOutSize = 0;

    bool isSha = caam_hash_alg_is_sha(algo); /* MDHA engine */
                                             /* how many bytes to read from context register
                                              * we need caam_hash_algo2ctx_size() to return
                                              * full context size (to be used for context restore in descriptor[3])
                                              */
    uint32_t caamCtxSz = caam_hash_algo2ctx_size(algo, 0 /* full context */);

    memcpy(descriptor, templateHash, sizeof(templateHash));

    /* MDHA is always Class 2 CHA, AESA configured at build time as Class 1 CHA */
    uint32_t class = isSha ? 0x04000000u : CAAM_AES_MAC_CLASS;

    /* add class to all commands that need it */
    descriptor[1] |= class;
    descriptor[3] |= class;
    descriptor[5] |= class;
    descriptor[6] |= class;
    descriptor[8] |= class;
    descriptor[10] |= class;

    /* add descriptor size */
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* kCAAM_AlgStateInit or kCAAM_AlgStateInitFinal needs to skip context load as there is no context */
    if ((algState == kCAAM_AlgStateInit) || (algState == kCAAM_AlgStateInitFinal))
    {
        if (isSha)
        {
            /* HEADER can jump directly to MDHA operation */
            descriptor[0] |= 0x00050000; /* JUMP to descriptor[5] */
        }
        else
        {
            /* load KEY, then directly to AESA MAC operation */
            descriptor[1] |= (keySize & 0x3FFu);
            descriptor[2] = keyAddr;
            descriptor[3] = 0xA0000002u; /* JUMP to descriptor[5] */
        }
    }
    else
    {
        if (isSha)
        {
            /* MDHA SHA in Update state skips loading the KEY, as MDHA SHA has no configurable key
             * HEADER can jump directly to context restore
             */
            descriptor[0] |= 0x00030000;       /* JUMP to descriptor[3] */
            /* descriptor[1] = 0xA0000002u; */ /* JUMP to descriptor[3] */
        }
        else
        {
            /* load KEY */
            descriptor[1] |= (keySize & 0x3FFu);
            descriptor[2] = keyAddr;

            /* XCBC-MAC K1 derived key has been ECB encrypted (black key)
             * so it needs decrypt
             */
            if (kCAAM_XcbcMac == algo)
            {
                descriptor[1] |= 1u << 22; /* ENC */
            }
        }

        /* context restore */
        descriptor[3] |= caamCtxSz;
        descriptor[4] = (uint32_t)context;
    }

    /* OPERATION:
     * alg MDHA or AESA
     * mode INITIALIZE or UPDATE or FINALIZE or INITIALIZE/FINALIZE in algState argument
     */

    /* ALGORITHM OPERATION | CLASS | alg | aai | algState */
    descriptor[5] |= caam_hash_algo2mode(algo, (uint32_t)algState << 2, &algOutSize);

    /* configure SGT */
    descriptor[6] |= (0xFFFFu & dataSize);
    if (kCAAM_HashSgtInternal == sgtType)
    {
        descriptor[7] = (uint32_t)&descriptor[kCAAM_HashDescriptorSgtIdx]; /* use SGT embedded in the job descriptor */
        memcpy(&descriptor[kCAAM_HashDescriptorSgtIdx], sgt, sizeof(caam_hash_internal_sgt_t));
    }
    else
    {
        descriptor[7] = (uint32_t)sgt;
    }

    /* save context: context switch init or running or result */
    if ((kCAAM_AlgStateFinal == algState) || (kCAAM_AlgStateInitFinal == algState))
    {
        if (outputSize)
        {
            if (algOutSize < *outputSize)
            {
                *outputSize = algOutSize;
            }
            else
            {
                algOutSize = *outputSize;
            }
        }
        caamCtxSz = algOutSize;
    }
    else
    {
        uint32_t how = (algState == kCAAM_AlgStateInit) ? 0 : 1; /* context switch needs full, then running */
        caamCtxSz = caam_hash_algo2ctx_size(algo, how);
    }
    descriptor[8] |= caamCtxSz;
    if ((kCAAM_AlgStateFinal == algState) || (kCAAM_AlgStateInitFinal == algState))
    {
        /* final result write to output */
        descriptor[9] = (uint32_t)output;
    }
    else
    {
        /* context switch write to ctxInternal */
        descriptor[9] = (uint32_t)context;
    }

    /* save the derived key K1 in XCBC-MAC. only if context switch. */
    if ((kCAAM_AlgStateInit == algState) && (kCAAM_XcbcMac == algo))
    {
        descriptor[10] |= (keySize & 0x3FFu);
        descriptor[11] = keyAddr;
    }
    else
    {
        descriptor[10] = descriptor[12]; /* always halt with status 0x0 (normal) */
    }

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

/*!
 * @brief Add uncomplete block (ctxInternal), then append new data (to current hash).
 *
 *
 */
static status_t caam_hash_append_data(caam_hash_ctx_internal_t *ctxInternal,
                                      const uint8_t *input,
                                      uint32_t inputSize,
                                      caam_algorithm_state_t algState,
                                      caam_desc_hash_t descriptor,
                                      uint32_t *numRemain,
                                      void *output,
                                      uint32_t *outputSize)
{
    caam_hash_internal_sgt_t sgt = {0};
    uint32_t num =
        caam_hash_sgt_insert(ctxInternal, input, inputSize, numRemain, algState, sgt, kCAAM_HashSgtEntryLast);
    return caam_hash_schedule_input_data(ctxInternal->base, ctxInternal->handle, ctxInternal->algo, sgt, num,
                                         kCAAM_HashSgtInternal, algState, descriptor, outputSize, output,
                                         &ctxInternal->word[0], (uint32_t)&ctxInternal->word[kCAAM_HashCtxKeyStartIdx],
                                         ctxInternal->word[kCAAM_HashCtxKeySize]);
}

status_t CAAM_HASH_Update(caam_hash_ctx_t *ctx, const uint8_t *input, uint32_t inputSize)
{
    caam_desc_hash_t descBuf;
    status_t status;
    caam_hash_ctx_internal_t *ctxInternal;
    bool isUpdateState;
    uint32_t numRemain = 0;

    /* compile time check for the correct structure size */
    BUILD_BUG(sizeof(caam_hash_ctx_t) < sizeof(caam_hash_ctx_internal_t));

    /* we do memcpy() input stream, up to buffer size
     * of 64 bytes. then if I have more I have to
     * 1) load Class 2 context
     * 2) schedule CAAM job with INITIALIZE or UPDATE mode (simple if only 64 bytes block is processed. SG table for 2
     * and more)
     * 3) in step 2 process all full 64 bytes blocks
     * 4) copy last not-full buffer size data to buffer.
     * 5) save Class 2 context
     */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    status = caam_hash_check_context(ctxInternal, input);
    if (kStatus_Success != status)
    {
        return status;
    }

    /* if we are still less than 64 bytes, keep only in context */
    if ((ctxInternal->blksz + inputSize) <= CAAM_HASH_BLOCK_SIZE)
    {
        memcpy((&ctxInternal->blk.b[0]) + ctxInternal->blksz, input, inputSize);
        ctxInternal->blksz += inputSize;
        return status;
    }
    else
    {
        isUpdateState = ctxInternal->state == kCAAM_HashUpdate;
        if (!isUpdateState)
        {
            /* Step 2: schedule CAAM job in INITIALIZE mode.
             */
            ctxInternal->state = kCAAM_HashUpdate;
            /* skip load context as there is no running context yet. */
            status = caam_hash_append_data(ctxInternal, input, inputSize, kCAAM_AlgStateInit, descBuf, &numRemain, NULL,
                                           NULL);
        }
    }

    if (kStatus_Success != status)
    {
        return status;
    }

    if (isUpdateState)
    {
        /* Step 2: schedule CAAM job in UPDATE mode.
         */

        /* process input data and save CAAM context to context structure */
        status =
            caam_hash_append_data(ctxInternal, input, inputSize, kCAAM_AlgStateUpdate, descBuf, &numRemain, NULL, NULL);
    }

    /* blocking wait? */
    status = CAAM_Wait(ctxInternal->base, ctxInternal->handle, descBuf, kCAAM_Blocking);

    /* after job is complete, copy numRemain bytes at the end of the input[] to the context */
    memcpy((&ctxInternal->blk.b[0]), input + inputSize - numRemain, numRemain);
    ctxInternal->blksz = numRemain;

    return status;
}

status_t CAAM_HASH_UpdateNonBlocking(caam_hash_ctx_t *ctx, const uint8_t *input, uint32_t inputSize)
{
    status_t status;

    /* compile time check for the correct structure size */
    BUILD_BUG(sizeof(caam_hash_ctx_t) < sizeof(caam_hash_ctx_internal_t));

    caam_hash_ctx_internal_t *ctxInternal;

    /* runtime input validity check */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    status = caam_hash_check_context(ctxInternal, input);
    if (kStatus_Success != status)
    {
        return status;
    }

    /* Add input data chunk to SGT */
    uint32_t currSgtEntry = ctxInternal->blksz;
    if (currSgtEntry >= kCAAM_HashSgtMaxCtxEntries)
    {
        return kStatus_InvalidArgument;
    }

    caam_sgt_entry_t *sgt = &((caam_sgt_entry_t *)ctxInternal)[currSgtEntry];
    caam_hash_sgt_insert(NULL, input, inputSize, NULL, kCAAM_AlgStateInitFinal, sgt,
                         kCAAM_HashSgtEntryNotLast /* not last. we don't know if this is the last chunk */);
    if (inputSize)
    {
        ctxInternal->blksz++;
    }

    return status;
}

status_t CAAM_HASH_Finish(caam_hash_ctx_t *ctx, uint8_t *output, uint32_t *outputSize)
{
    status_t status;
    caam_hash_ctx_internal_t *ctxInternal;
    caam_desc_hash_t descBuf;
    caam_algorithm_state_t algState;

    /* runtime input validity check */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    status = caam_hash_check_context(ctxInternal, output);
    if (kStatus_Success != status)
    {
        return status;
    }

    /* determine algorithm state to configure
     * based on prior processing.
     * If at least one full block has been processed during HASH_Update() then the state in ctxInternal
     * will be set to kCAAM_HashUpdate and so we will configure FINALIZE algorithm state.
     * Otherwise there is data only in the ctxInternal that we can process in INITIALIZE/FINALIZE.
     */
    if (ctxInternal->state == kCAAM_HashInit)
    {
        algState = kCAAM_AlgStateInitFinal;
    }
    else
    {
        algState = kCAAM_AlgStateFinal;
    }

    status = caam_hash_append_data(
        ctxInternal, NULL, 0, /* we process only blksz bytes in ctxInternal, so giving NULL and zero size here */
        algState, descBuf, NULL, output, outputSize);
    if (kStatus_Success != status)
    {
        return status;
    }

    /* blocking wait */
    status = CAAM_Wait(ctxInternal->base, ctxInternal->handle, descBuf, kCAAM_Blocking);
    memset(ctx, 0, sizeof(caam_hash_ctx_t));
    return status;
}

status_t CAAM_HASH_FinishNonBlocking(caam_hash_ctx_t *ctx,
                                     caam_desc_hash_t descriptor,
                                     uint8_t *output,
                                     uint32_t *outputSize)
{
    status_t status;
    caam_hash_ctx_internal_t *ctxInternal;

    /* runtime input validity check */
    ctxInternal = (caam_hash_ctx_internal_t *)ctx;
    status = caam_hash_check_context(ctxInternal, output);
    if (kStatus_Success != status)
    {
        return status;
    }

    uint32_t currSgtEntry = ctxInternal->blksz;
    if (currSgtEntry > kCAAM_HashSgtMaxCtxEntries)
    {
        return kStatus_InvalidArgument;
    }

    caam_sgt_entry_t *sgt = &((caam_sgt_entry_t *)ctxInternal)[0];

    /* mark currSgtEntry with Final Bit */
    int i;
    uint32_t totalLength = 0;
    for (i = 0; i < currSgtEntry; i++)
    {
        totalLength += sgt[i].length;
    }
    sgt[currSgtEntry].length |= 0x40000000u; /* Final SG entry */

    status = caam_hash_schedule_input_data(ctxInternal->base, ctxInternal->handle, ctxInternal->algo, sgt, totalLength,
                                           kCAAM_HashSgtExternal, kCAAM_AlgStateInitFinal, descriptor, outputSize,
                                           output, NULL, (uint32_t)&ctxInternal->word[kCAAM_HashCtxKeyStartIdx],
                                           ctxInternal->word[kCAAM_HashCtxKeySize]);
    return kStatus_Success;
}

status_t CAAM_HASH(CAAM_Type *base,
                   caam_handle_t *handle,
                   caam_hash_algo_t algo,
                   const uint8_t *input,
                   uint32_t inputSize,
                   const uint8_t *key,
                   uint32_t keySize,
                   uint8_t *output,
                   uint32_t *outputSize)
{
    status_t status;
    caam_desc_hash_t descBuf;

    status = CAAM_HASH_NonBlocking(base, handle, descBuf, algo, input, inputSize, key, keySize, output, outputSize);
    if (kStatus_Success != status)
    {
        return status;
    }

    status = CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
    return status;
}

status_t CAAM_HASH_NonBlocking(CAAM_Type *base,
                               caam_handle_t *handle,
                               caam_desc_hash_t descriptor,
                               caam_hash_algo_t algo,
                               const uint8_t *input,
                               uint32_t inputSize,
                               const uint8_t *key,
                               uint32_t keySize,
                               uint8_t *output,
                               uint32_t *outputSize)
{
    status_t status;
    caam_algorithm_state_t algState;
    caam_hash_internal_sgt_t sgt = {0};

    algState = kCAAM_AlgStateInitFinal;
    uint32_t num = caam_hash_sgt_insert(NULL,             /* no ctxInternal data to pre-pend before input data chunk */
                                        input, inputSize, /* data and size in bytes */
                                        NULL, /* all data is processed during kCAAM_AlgStateInitFinal, nothing remain */
                                        algState, sgt, kCAAM_HashSgtEntryLast); /* sgt table, entry 0 word 0 */

    /* schedule the request at CAAM */
    status = caam_hash_schedule_input_data(base, handle, algo, sgt, num, kCAAM_HashSgtInternal, algState, descriptor,
                                           outputSize, output, NULL, (uint32_t)key, keySize);
    return status;
}

/*******************************************************************************
 * RNG Code public
 ******************************************************************************/

status_t CAAM_RNG_GetDefaultConfig(caam_rng_config_t *config)
{
    status_t status;
    if (config)
    {
        config->autoReseedInterval = 0; /* zero means hardware default of 10.000.000 will be used */
        config->personalString = NULL;
        status = kStatus_Success;
    }
    else
    {
        status = kStatus_InvalidArgument;
    }

    return status;
}

status_t CAAM_RNG_Init(CAAM_Type *base,
                       caam_handle_t *handle,
                       caam_rng_state_handle_t stateHandle,
                       const caam_rng_config_t *config)
{
    status_t status;

    /* create job descriptor */
    uint32_t rngInstantiate[] = {
        /* 00 */ 0xB0800006u,
        /* 01 */ 0x12200020u, /* LOAD 32 bytes of  to Class 1 Context Register. Offset 0 bytes. */
        /* 02 */ (uint32_t)config->personalString,
        /* 03 */ 0x12820004u,                /* LOAD Immediate 4 bytes to Class 1 Data Size Register. */
        /* 04 */ config->autoReseedInterval, /* value for the Class 1 Data Size Register */
        /* 05 */ 0x82500004u,                /* RNG instantiate state handle */
    };

    if (kCAAM_RngStateHandle1 == stateHandle)
    {
        rngInstantiate[5] |= 1u << 4;
    }

    /* default auto reseed interval */
    if (config->autoReseedInterval == 0)
    {
        rngInstantiate[3] = 0xA0000002u; /* jump to current index + 2 (=5) */
    }

    /* optional personalization string present */
    if (config->personalString)
    {
        rngInstantiate[5] |= 1u << 10; /* set PS bit in ALG OPERATION (AS=01 Instantiate) */
    }
    else
    {
        rngInstantiate[1] = 0xA0000002u; /* jump to current index + 2 (=3) */
    }

    /* schedule the job and block wait for result */
    do
    {
        status = caam_in_job_ring_add(base, handle->jobRing, &rngInstantiate[0]);
    } while (status != kStatus_Success);

    status = CAAM_Wait(base, handle, &rngInstantiate[0], kCAAM_Blocking);
    return status;
}

status_t CAAM_RNG_Deinit(CAAM_Type *base, caam_handle_t *handle, caam_rng_state_handle_t stateHandle)
{
    status_t status;

    /* create job descriptor */
    uint32_t rngUninstantiate[] = {
        0xB0800002u, /* HEADER */
        0x8250000Du, /* ALG OPERATION: RNG uninstantiate state handle (AS=11 Uninstantiate) */
    };

    if (kCAAM_RngStateHandle1 == stateHandle)
    {
        rngUninstantiate[1] |= 1u << 4;
    }

    /* schedule the job and block wait for result */
    do
    {
        status = caam_in_job_ring_add(base, handle->jobRing, &rngUninstantiate[0]);
    } while (status != kStatus_Success);

    status = CAAM_Wait(base, handle, &rngUninstantiate[0], kCAAM_Blocking);

    return status;
}

status_t CAAM_RNG_GenerateSecureKey(CAAM_Type *base, caam_handle_t *handle, caam_rng_generic256_t additionalEntropy)
{
    status_t status;

    /* create job descriptor */
    uint32_t rngGeSeckey[] = {
        0xB0800004u,
        /* 01 */ 0x12200020u, /* LOAD 32 bytes of  to Class 1 Context Register. Offset 0 bytes. */
        /* 02 */ (uint32_t)additionalEntropy, 0x82501000u, /* set SK bit in ALG OPERATION (AS=00 Generate) */
    };

    /* optional additional input included */
    if (additionalEntropy)
    {
        rngGeSeckey[3] |= 1u << 11; /* set AI bit in ALG OPERATION */
    }
    else
    {
        rngGeSeckey[1] = 0xA0000002u; /* jump to current index + 2 (=3) */
    }

    /* schedule the job and block wait for result */
    do
    {
        status = caam_in_job_ring_add(base, handle->jobRing, &rngGeSeckey[0]);
    } while (status != kStatus_Success);

    status = CAAM_Wait(base, handle, &rngGeSeckey[0], kCAAM_Blocking);
    return status;
}

status_t CAAM_RNG_Reseed(CAAM_Type *base,
                         caam_handle_t *handle,
                         caam_rng_state_handle_t stateHandle,
                         caam_rng_generic256_t additionalEntropy)
{
    status_t status;

    /* create job descriptor */
    uint32_t rngReseed[] = {
        /* 00 */ 0xB0800004u, /* HEADER */
        /* 01 */ 0x12200020u, /* LOAD 32 bytes of  to Class 1 Context Register. Offset 0 bytes. */
        /* 02 */ (uint32_t)additionalEntropy,
        /* 03 */ 0x8250000Au, /* ALG OPERATION: RNG reseed state handle (AS=10 Reseed) */
    };

    /* optional additional input included */
    if (additionalEntropy)
    {
        rngReseed[3] |= 1u << 11; /* set AI bit in ALG OPERATION */
    }
    else
    {
        rngReseed[1] = 0xA0000002u; /* jump to current index + 2 (=3) */
    }

    /* select state handle */
    if (kCAAM_RngStateHandle1 == stateHandle)
    {
        rngReseed[3] |= 1u << 4;
    }

    /* schedule the job and block wait for result */
    do
    {
        status = caam_in_job_ring_add(base, handle->jobRing, &rngReseed[0]);
    } while (status != kStatus_Success);

    status = CAAM_Wait(base, handle, &rngReseed[0], kCAAM_Blocking);
    return status;
}

status_t CAAM_RNG_GetRandomData(CAAM_Type *base,
                                caam_handle_t *handle,
                                caam_rng_state_handle_t stateHandle,
                                void *data,
                                size_t dataSize,
                                caam_rng_random_type_t dataType,
                                caam_rng_generic256_t additionalEntropy)
{
    status_t status;
    caam_desc_rng_t descBuf;

    do
    {
        status = CAAM_RNG_GetRandomDataNonBlocking(base, handle, stateHandle, descBuf, data, dataSize, dataType,
                                                   additionalEntropy);
    } while (status == kStatus_Again);

    if (kStatus_Success != status)
    {
        return status;
    }

    status = CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
    return status;
}

static const uint32_t templateRng[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x12200020u, /* LOAD 32 bytes of  to Class 1 Context Register. Offset 0 bytes. */
    /* 02 */ 0x00000000u, /* place: additional input address */
    /* 03 */ 0x60300000u, /* FIFO STORE message */
    /* 04 */ 0x00000000u, /* place: destination address */
    /* 05 */ 0x82500004u, /* RNG generate */
};

status_t CAAM_RNG_GetRandomDataNonBlocking(CAAM_Type *base,
                                           caam_handle_t *handle,
                                           caam_rng_state_handle_t stateHandle,
                                           caam_desc_rng_t descriptor,
                                           void *data,
                                           size_t dataSize,
                                           caam_rng_random_type_t dataType,
                                           caam_rng_generic256_t additionalEntropy)
{
    /* create job descriptor */
    BUILD_BUG(sizeof(templateRng) != sizeof(caam_desc_rng_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateRng);

    memcpy(descriptor, templateRng, sizeof(templateRng));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);

    /* optional additional input included */
    if (additionalEntropy)
    {
        descriptor[2] = (uint32_t)additionalEntropy;
        descriptor[5] |= 1u << 11; /* set AI bit in ALG OPERATION */
    }
    else
    {
        descriptor[0] |= 3u << 16; /* start at index 3 */
    }

    descriptor[3] |= (dataSize & 0xFFFFu);
    descriptor[4] = (uint32_t)data;

    /* select state handle */
    if (kCAAM_RngStateHandle1 == stateHandle)
    {
        descriptor[5] |= 1u << 4;
    }

    /* configure type of data */
    if (dataType == kCAAM_RngDataNonZero)
    {
        descriptor[5] |= 1u << 8; /* set NZB bit in ALG OPERATION */
    }

    if (dataType == kCAAM_RngDataOddParity)
    {
        descriptor[5] |= 1u << 9; /* set OBP bit in ALG OPERATION */
    }

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

static const uint32_t templateCipherDes[] = {
    /* 00 */ 0xB0800000u, /* HEADER */
    /* 01 */ 0x02800000u, /* KEY Class 1 IMM */
    /* 02 */ 0x00000000u, /* IMM key1 0-3 */
    /* 03 */ 0x00000000u, /* IMM key1 4-8 */
    /* 04 */ 0x00000000u, /* IMM key2 0-3 */
    /* 05 */ 0x00000000u, /* IMM key2 4-8 */
    /* 06 */ 0x00000000u, /* IMM key3 0-3 */
    /* 07 */ 0x00000000u, /* IMM key3 4-8 */
    /* 08 */ 0x12200008u, /* LOAD 8 bytes of iv to Class 1 Context Register */
    /* 09 */ 0x00000000u, /* place: iv address */
    /* 10 */ 0x22130000u, /* FIFO LOAD Message */
    /* 11 */ 0x00000000u, /* place: source address */
    /* 12 */ 0x60300000u, /* FIFO STORE Message */
    /* 13 */ 0x00000000u, /* place: destination address */
    /* 14 */ 0x82200000u, /* OPERATION: DES Decrypt, AS = zeroes, AAI = zeroes (CTR) */
};

/*******************************************************************************
 * DES Code public
 ******************************************************************************/

status_t CAAM_DES_EncryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES_EncryptEcbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, key);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_EncryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000006u; /* ECB has no context, jump to currIdx+6 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x201u; /* add ENC bit to specify Encrypt OPERATION, AAI = 20h */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_DecryptEcb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES_DecryptEcbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, key);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_DecryptEcbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000006u; /* ECB has no context, jump to currIdx+6 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x20u << 4; /* AAI = 20h */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_EncryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_EncryptCbcNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_EncryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_DecryptCbc(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_DecryptCbcNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_DecryptCbcNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_EncryptCfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_EncryptCfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_EncryptCfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_DecryptCfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_DecryptCfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_DecryptCfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_EncryptOfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *plaintext,
                             uint8_t *ciphertext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_EncryptOfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_EncryptOfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *plaintext,
                                        uint8_t *ciphertext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES_DecryptOfb(CAAM_Type *base,
                             caam_handle_t *handle,
                             const uint8_t *ciphertext,
                             uint8_t *plaintext,
                             uint32_t size,
                             const uint8_t iv[CAAM_DES_IV_SIZE],
                             const uint8_t key[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES_DecryptOfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES_DecryptOfbNonBlocking(CAAM_Type *base,
                                        caam_handle_t *handle,
                                        caam_desc_cipher_des_t descriptor,
                                        const uint8_t *ciphertext,
                                        uint8_t *plaintext,
                                        uint32_t size,
                                        const uint8_t iv[CAAM_DES_IV_SIZE],
                                        const uint8_t key[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key, CAAM_DES_KEY_SIZE);
    descriptor[4] = 0xA0000004u; /* context, jump to currIdx+4 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_EncryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES2_EncryptEcbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, key1, key2);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_EncryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000004u; /* ECB has no context, jump to currIdx+4 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x201u;  /* add ENC bit to specify Encrypt OPERATION, AAI = 20h */
    descriptor[14] |= 0x10000; /* 3DES */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_DecryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES2_DecryptEcbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, key1, key2);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_DecryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000004u; /* ECB has no context, jump to currIdx+4 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x20u << 4; /* AAI = 20h */
    descriptor[14] |= 0x10000;    /* 3DES */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_EncryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_EncryptCbcNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_EncryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_DecryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_DecryptCbcNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_DecryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_EncryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_EncryptCfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_EncryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_DecryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_DecryptCfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_DecryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_EncryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_EncryptOfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_EncryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES2_DecryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status = CAAM_DES2_DecryptOfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES2_DecryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 2 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    descriptor[6] = 0xA0000002u; /* context, jump to currIdx+2 = 8 (LOAD) */
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_EncryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES3_EncryptEcbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, key1, key2, key3);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_EncryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[8] = 0xA0000002u; /* ECB has no context, jump to currIdx+2 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x201u;  /* add ENC bit to specify Encrypt OPERATION, AAI = 20h */
    descriptor[14] |= 0x10000; /* 3DES */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_DecryptEcb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    caam_desc_cipher_des_t descBuf;
    status_t status;

    status = CAAM_DES3_DecryptEcbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, key1, key2, key3);
    if (status)
    {
        return status;
    }
    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_DecryptEcbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[8] = 0xA0000002u; /* ECB has no context, jump to currIdx+2 = 10 (FIFO LOAD) */
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x20u << 4; /* AAI = 20h */
    descriptor[14] |= 0x10000;    /* 3DES */

    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_EncryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_EncryptCbcNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2, key3);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_EncryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_DecryptCbc(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_DecryptCbcNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2, key3);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_DecryptCbcNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x10u << 4; /* AAI = 10h */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_EncryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_EncryptCfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2, key3);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_EncryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_DecryptCfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_DecryptCfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2, key3);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_DecryptCfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x30u << 4; /* AAI = 30h = CFB */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_EncryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *plaintext,
                              uint8_t *ciphertext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_EncryptOfbNonBlocking(base, handle, descBuf, plaintext, ciphertext, size, iv, key1, key2, key3);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_EncryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *plaintext,
                                         uint8_t *ciphertext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)plaintext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)ciphertext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 1u;         /* add ENC bit to specify Encrypt OPERATION */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}

status_t CAAM_DES3_DecryptOfb(CAAM_Type *base,
                              caam_handle_t *handle,
                              const uint8_t *ciphertext,
                              uint8_t *plaintext,
                              uint32_t size,
                              const uint8_t iv[CAAM_DES_IV_SIZE],
                              const uint8_t key1[CAAM_DES_KEY_SIZE],
                              const uint8_t key2[CAAM_DES_KEY_SIZE],
                              const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    status_t status;
    caam_desc_cipher_des_t descBuf;

    do
    {
        status =
            CAAM_DES3_DecryptOfbNonBlocking(base, handle, descBuf, ciphertext, plaintext, size, iv, key1, key2, key3);
    } while (status == kStatus_Again);

    return CAAM_Wait(base, handle, descBuf, kCAAM_Blocking);
}

status_t CAAM_DES3_DecryptOfbNonBlocking(CAAM_Type *base,
                                         caam_handle_t *handle,
                                         caam_desc_cipher_des_t descriptor,
                                         const uint8_t *ciphertext,
                                         uint8_t *plaintext,
                                         uint32_t size,
                                         const uint8_t iv[CAAM_DES_IV_SIZE],
                                         const uint8_t key1[CAAM_DES_KEY_SIZE],
                                         const uint8_t key2[CAAM_DES_KEY_SIZE],
                                         const uint8_t key3[CAAM_DES_KEY_SIZE])
{
    BUILD_BUG(sizeof(templateCipherDes) != sizeof(caam_desc_cipher_des_t));
    uint32_t descriptorSize = ARRAY_SIZE(templateCipherDes);

    memcpy(descriptor, templateCipherDes, sizeof(templateCipherDes));
    descriptor[0] |= (descriptorSize & 0x0000007Fu);
    descriptor[1] |= 3 * CAAM_DES_KEY_SIZE;
    memcpy(&descriptor[2], key1, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[4], key2, CAAM_DES_KEY_SIZE);
    memcpy(&descriptor[6], key3, CAAM_DES_KEY_SIZE);
    descriptor[9] = (uint32_t)iv;
    descriptor[10] |= (size & 0x0000FFFFu);
    descriptor[11] = (uint32_t)ciphertext;
    descriptor[12] |= (size & 0x0000FFFFu);
    descriptor[13] = (uint32_t)plaintext;
    descriptor[14] |= 0x40u << 4; /* AAI = 40h = OFB */
    descriptor[14] |= 0x10000;    /* 3DES */
    return caam_in_job_ring_add(base, handle->jobRing, &descriptor[0]);
}
