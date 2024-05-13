#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include "log.h"

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define SEG_SOI 0xD8  // start of image
#define SEG_APP0 0xE0 // application 0
#define SEG_SOF0 0xC0 // start of frame 0(0: baseline)
#define SEG_DQT 0xDB  // define quantization table
#define SEG_DHT 0xC4  // define huffman table
#define SEG_SOS 0xDA  // start of scan
#define SEG_EOI 0xD9  // end of image

const char *marker_name(int seg_id)
{
    switch (seg_id)
    {
    case SEG_SOI: return "SOI";
    case SEG_APP0: return "APP0";
    case SEG_SOF0: return "SOF0";
    case SEG_DQT: return "DQT";
    case SEG_DHT: return "DHT";
    case SEG_SOS: return "SOS";
    case SEG_EOI: return "EOI";
    default:
        log_("[%x]\n", seg_id);
        return "Unknown";
    }
}

//          |区段头0xFFDB|段长|量化值大小|id|数据    |
// 长度(bit)|16          |16  |4         |4 |段长指定|
struct define_quantization_table
{
    uint8_t *ptr;          // 包含0xFFDB
    int length;            // 不包含0xFFDB，包含长度字节的总长度
    int quantization_size; // 标识字节的高4位，标识每个量化值大小，0:1byte/1:2bytes
    int id;                // 标识字节的低4位，id可为0/1/2/3
};

struct define_huffman_table_code_item
{
    uint16_t code;
    uint16_t mask;
    uint8_t value;
};

// [TODO] 在一个DHT中可能有多个huffman表
//          |区段头0xFFC4|段长|AC/DC|表号|霍夫曼树叶子节点个数表|数据        |
// 长度(bit)|16          |16  |4    |4   |128                   |叶子节点个数|
struct define_huffman_table
{
    uint8_t *ptr;                                 // 包含0xFFC4
    int length;                                   // 不包含0xFFC4，包含长度字节的总长度
    int ac_dc_type;                               // 直流0/交流1
    int table_id;                                 // 表号，最低位有效，高3位固定0
    uint8_t leave_counts[16];                     // 霍夫曼表码字长度对应的叶子节点个数
    int leave_count_total;                        // 叶子节点总数，即码字总数
    struct define_huffman_table_code_item *items; // 每个码字的信息
};

//          |颜色分量id|水平采样率|垂直采样率|量化表id|
// 长度(bit)|8         |4         |4         |8       |
struct start_of_frame_0_channel_info
{
    int color_id;               // 颜色分量id：1:Y/2:Cb/3:Cr
    int horizontal_sample_rate; // 水平采样率，取值1/2/3/4
    int vertical_sample_rate;   // 垂直采样率，取值1/2/3/4
    int dqt_table_id;           // 量化表id
};

//          |区段头0xFFC0|段长|精度|图像高|图像宽|颜色分量数目|各颜色分量信息|
// 长度(bit)|16          |16  |8   |16    |16    |8           |72            |
// 各颜色分量详细信息，以下表重复3次
struct start_of_frame_0
{
    uint8_t *ptr;
    int length;
    int accuracy;
    int height;
    int width;
    int color_channel_count;
    struct start_of_frame_0_channel_info channel_info[3];
};

//          |颜色分量id|直流霍夫曼表id|交流霍夫曼表id|
// 长度(bit)|8         |4             |4             |
struct start_of_scan_channel_info
{
    int color_id;
    int dc_dht_id;
    int ac_dht_id;
};

//          |区段头0xFFDA|段长|颜色分量数目|对应霍夫曼表|baseline无用|
// 长度(bit)|16          |16  |8           |48          |24          |
struct start_of_scan
{
    uint8_t *ptr;
    int length;
    int color_channel_count;
    struct start_of_scan_channel_info channel_info[3];
    uint8_t not_baseline_0;
    uint8_t not_baseline_1;
    uint8_t not_baseline_2;
};

struct context
{
    FILE *fp;
    int length;
    uint8_t *buffer;
    uint8_t *ptr; // running pointer

    uint8_t *ptr_SOI;
    uint8_t **ptr_APP0s;
    int count_APP0s;
    struct start_of_frame_0 SOF0;
    struct define_quantization_table *DQTs;
    int count_DQTs;
    struct define_huffman_table *DHTs;
    int count_DHTs;
    struct start_of_scan SOS;
    uint8_t *compress_data;
    uint8_t *ptr_EOI;
};

void usage(const char *name)
{
    log_("%s <filename>\n", name);
}

uint8_t get_byte(struct context *ctx)
{
    return *ctx->ptr++;
}

uint16_t get_2bytes(struct context *ctx)
{
    return get_byte(ctx) << 8 | get_byte(ctx);
}

uint16_t read_bits(struct context *ctx, int start, int length)
{
    uint32_t v = *(uint32_t *)(ctx->compress_data + start / 8);
    uint32_t mask = ((0x01 << length) - 1) << start;
    uint16_t result = (uint16_t)((v & mask) >> start);

    return result;
}

void read_DQT(struct context *ctx)
{
    ctx->DQTs = realloc(ctx->DQTs, (++ctx->count_DQTs) * sizeof(struct define_quantization_table));
    struct define_quantization_table *dqt = &ctx->DQTs[ctx->count_DQTs - 1];

    dqt->ptr = ctx->ptr - 2;
    dqt->length = get_2bytes(ctx);
    uint8_t byte = get_byte(ctx);
    dqt->quantization_size = (byte >> 4) & 0x0F;
    dqt->id = byte & 0x0F;
}

void dump_DQTs(struct context *ctx)
{
    printf("DQT\n");
    printf(" virtual addr\toffset\tlength\tquan size\tid\n");
    for (int i = 0; i < ctx->count_DQTs; ++i)
    {
        struct define_quantization_table *dqt = &ctx->DQTs[i];
        printf(" %p\t%ld\t%d\t%d\t\t%d\n", dqt->ptr, dqt->ptr - ctx->buffer, dqt->length, dqt->quantization_size, dqt->id);
    }
    printf("\n");
}

void read_DHT(struct context *ctx)
{
    ctx->DHTs = realloc(ctx->DHTs, (++ctx->count_DHTs) * sizeof(struct define_huffman_table));
    struct define_huffman_table *dht = &ctx->DHTs[ctx->count_DHTs - 1];

    dht->ptr = ctx->ptr - 2;
    dht->length = get_2bytes(ctx);
    uint8_t byte = get_byte(ctx);
    dht->ac_dc_type = (byte >> 4) & 0x0F;
    dht->table_id = byte & 0x0F;

    struct define_huffman_table_code_item item = {0x0000, 0x0001};
    // uint8_t test_dht_counts[] = {0x00, 0x02, 0x02, 0x00, 0x05, 0x01, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int item_index = 0;
    for (int i = 0; i < 16; ++i)
    {
        dht->leave_counts[i] = get_byte(ctx); // 该层个数
        if (dht->leave_counts[i] != 0)
        {
            // dht->leave_counts[i] = test_dht_counts[i];
            dht->leave_count_total += dht->leave_counts[i];
            dht->items = realloc(dht->items, dht->leave_count_total * sizeof(struct define_huffman_table_code_item));
            for (int j = 0; j < dht->leave_counts[i]; ++j) // 计算该层每个码字
            {
                dht->items[item_index++] = item;
                // printf("len: %d, id: %d, code: %x, mask: %x\n", i + 1, j, item.code, item.mask);
                ++item.code; // 存在数目的情况下码字要先+1
            }
        }
        item.code <<= 1; // 再左移
        item.mask <<= 1; // mask要先左移
        ++item.mask;     // 再+1
    }
    for (int i = 0; i < dht->leave_count_total; ++i)
    {
        dht->items[i].value = get_byte(ctx);
    }
    if (dht->length - 2 - 1 - 16 != dht->leave_count_total)
    {
        log_("DHT %d seems not simple\n", ctx->count_DHTs - 1);
    }

    // for (int i = 0; i < dht->leave_count_total; ++i)
    // {
    //     printf("code: %x, mask: %x, value: %02x\n", dht->items[i].code, dht->items[i].mask, dht->items[i].value);
    // }
}

void dump_DHTs(struct context *ctx)
{
    printf("DHT\n");
    printf(" virtual addr\toffset\tlength\tac/dc\ttable id\n");
    for (int i = 0; i < ctx->count_DHTs; ++i)
    {
        struct define_huffman_table *dht = &ctx->DHTs[i];
        printf(" %p\t%ld\t%d\t%d\t%d\n", dht->ptr, dht->ptr - ctx->buffer, dht->length, dht->ac_dc_type, dht->table_id);
        printf("  code\tvalue\tmask\n");
        for (int j = 0; j < min(dht->leave_count_total, 20); ++j)
        {
            printf("  %x\t%x\t%x\n", dht->items[j].code, dht->items[j].value, dht->items[j].mask);
        }
        if (dht->leave_count_total > 20)
        {
            printf("  ...\t...\t...\n");
        }
        printf("\n");
    }
}

void read_SOF0(struct context *ctx)
{
    struct start_of_frame_0 *sof0 = &ctx->SOF0;

    sof0->ptr = ctx->ptr - 2;
    sof0->length = get_2bytes(ctx);
    sof0->accuracy = get_byte(ctx);
    sof0->height = get_2bytes(ctx);
    sof0->width = get_2bytes(ctx);
    sof0->color_channel_count = get_byte(ctx);
    for (int i = 0; i < ctx->SOF0.color_channel_count; ++i)
    {
        struct start_of_frame_0_channel_info *ci = &sof0->channel_info[i];

        ci->color_id = get_byte(ctx);
        uint8_t byte = get_byte(ctx);
        ci->horizontal_sample_rate = (byte >> 4) & 0x0F;
        ci->vertical_sample_rate = byte & 0x0F;
        ci->dqt_table_id = get_byte(ctx);
    }
}

void dump_SOF0(struct context *ctx)
{
    struct start_of_frame_0 *sof0 = &ctx->SOF0;
    printf("SOF0\n");
    printf(" virtual addr\toffset\tlength\taccuracy\tresolution\tchannel count(3)\n");
    printf(" %p\t%ld\t%d\t%d\t\t%dx%d\t\t%d\n",
        sof0->ptr, sof0->ptr - ctx->buffer, sof0->length, sof0->accuracy, sof0->width, sof0->height, sof0->color_channel_count);
    printf("  color id\thorizontal sample rate\tvertical sample rate\tdqt id\n");
    for (int i = 0; i < sof0->color_channel_count; ++i)
    {
        struct start_of_frame_0_channel_info *ci = &sof0->channel_info[i];
        printf("  %d\t\t%d\t\t\t%d\t\t\t%d\n",
            ci->color_id, ci->horizontal_sample_rate, ci->vertical_sample_rate, ci->dqt_table_id);
    }
    printf("\n");
}

void read_SOS(struct context *ctx)
{
    struct start_of_scan *sos = &ctx->SOS;

    sos->ptr = ctx->ptr - 2;
    sos->length = get_2bytes(ctx);
    sos->color_channel_count = get_byte(ctx);
    for (int i = 0; i < 3; ++i)
    {
        struct start_of_scan_channel_info *ci = &sos->channel_info[i];

        ci->color_id = get_byte(ctx);
        uint8_t byte = get_byte(ctx);
        ci->dc_dht_id = (byte >> 4) & 0x0F;
        ci->ac_dht_id = byte & 0x0F;
    }
    sos->not_baseline_0 = get_byte(ctx);
    sos->not_baseline_1 = get_byte(ctx);
    sos->not_baseline_2 = get_byte(ctx);
    ctx->compress_data = ctx->ptr;
}

void dump_SOS(struct context *ctx)
{
    struct start_of_scan *sos = &ctx->SOS;

    printf("SOS\n");
    printf(" virtual addr\toffset\tlength\tcolor channel count\t[not for baseline]\n");
    printf(" %p\t%ld\t%d\t%d\t\t\t0x%02x%02x%02x\n",
        sos->ptr, sos->ptr - ctx->buffer, sos->length, sos->color_channel_count, sos->not_baseline_0, sos->not_baseline_1, sos->not_baseline_2);
    printf("  color id\tdc dht id\tac dht id\n");
    for (int i = 0; i < sos->color_channel_count; ++i)
    {
        struct start_of_scan_channel_info *ci = &sos->channel_info[i];
        printf("  %d\t\t%d\t\t%d\n", ci->color_id, ci->dc_dht_id, ci->ac_dht_id);
    }
    printf("\n");
}

void read_block() {}

void read_MCU(struct context *ctx)
{
    for (int i = 0; i < 16; ++i)
        printf("%02x ", *(ctx->compress_data + i));
    printf("\n");
    int start = 0;
    int length = 2;
    while (1)
    {
        uint16_t code = read_bits(ctx, start, length);

        int found = 0;
        for (int i = 0; i < ctx->DHTs[0].leave_count_total; ++i)
        {
            if (code == ctx->DHTs[0].items[i].code)
            {
                printf("code: %x, value: %x\n", ctx->DHTs[0].items[i].code, ctx->DHTs[0].items[i].value);
                found = 1;
                break;
            }
        }

        if (!found)
        {
            length++;
        }
        else
        {
            start += length;
            length = 2;
        }
    }
}

int main(int argc, char *argv[])
{
    struct context *ctx = calloc(1, sizeof(struct context));
    if (!ctx)
    {
        log_("calloc failed: %s\n", strerror(errno));
        goto error;
    }

    if (argc < 2)
    {
        usage(argv[0]);
        goto error;
    }

    ctx->fp = fopen(argv[1], "rb");
    if (!ctx->fp)
    {
        log_("fopen `%s` failed: %s\n", argv[1], strerror(errno));
        goto error;
    }

    fseek(ctx->fp, 0, SEEK_END);
    ctx->length = ftell(ctx->fp);
    rewind(ctx->fp);

    ctx->buffer = calloc(ctx->length, sizeof(uint8_t));
    if (!ctx->buffer)
    {
        log_("calloc failed: %s\n", strerror(errno));
        goto error;
    }
    ctx->ptr = ctx->buffer;
    fread(ctx->buffer, 1, ctx->length, ctx->fp);

#define add_seg(type)                                                                                   \
    do                                                                                                  \
    {                                                                                                   \
        ctx->ptr_##type##s = realloc(ctx->ptr_##type##s, (++ctx->count_##type##s) * sizeof(uint8_t *)); \
        ctx->ptr_##type##s[ctx->count_##type##s - 1] = ctx->ptr - 2;                                    \
    }                                                                                                   \
    while (0)

    while (ctx->ptr - ctx->buffer < ctx->length)
    {
        // 各段均以0xFF起始
        if (get_byte(ctx) != 0xFF)
            continue;

        // 下一个字节为区段marker
        uint8_t byte = get_byte(ctx);

        switch (byte)
        {
        // 若为0x00则是普通数据，要求图像数据中所有0xFF都要接一个0x00
        case 0x00: break;
        case SEG_SOI: ctx->ptr_SOI = ctx->ptr - 2; break;
        case SEG_APP0: add_seg(APP0); break;
        case SEG_SOF0: read_SOF0(ctx); break;
        case SEG_DQT: read_DQT(ctx); break;
        case SEG_DHT: read_DHT(ctx); break;
        case SEG_SOS: read_SOS(ctx); break;
        case SEG_EOI: ctx->ptr_EOI = ctx->ptr - 2; break;
        }

        // log_("marker: %lu %s\n", ctx->ptr - ctx->buffer, marker_name(byte));
    }

    dump_DQTs(ctx);
    dump_DHTs(ctx);
    dump_SOF0(ctx);
    dump_SOS(ctx);

    read_MCU(ctx);

error:
#define free_seg(type)                \
    do                                \
    {                                 \
        if (ctx->ptr_##type##s)       \
            free(ctx->ptr_##type##s); \
        ctx->count_##type##s = 0;     \
    }                                 \
    while (0)

    if (!ctx)
        exit(0);

    free_seg(APP0);
    // free_seg(SOF0);
    if (ctx->DQTs)
        free(ctx->DQTs);
    if (ctx->DHTs)
        free(ctx->DHTs);
    // free_seg(SOS);
    if (ctx->buffer)
        free(ctx->buffer);
    if (ctx->fp)
        fclose(ctx->fp);
}