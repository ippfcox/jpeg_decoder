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

struct block
{
    int coefficient[64];
};

struct context
{
    FILE *fp;
    int length;
    uint8_t *buffer;
    uint8_t *ptr;      // running pointer
    size_t bit_offset; // running

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

uint8_t get_bit(struct context *ctx)
{
    int bit_byte_offset = ctx->bit_offset % 8; // 在该字节中的位偏移量
    int byte_offset = ctx->bit_offset / 8;     // 所在字节的偏移量
    if (*(ctx->compress_data + byte_offset) == 0x00 && *(ctx->compress_data + byte_offset - 1) == 0xFF)
    {
        log_("jump 0xFF00, offset: %lx-%ld\n", ctx->compress_data - ctx->buffer, ctx->bit_offset);
        ctx->bit_offset += 8;
    }

    return ((*(ctx->compress_data + ctx->bit_offset / 8)) >> (7 - ctx->bit_offset++ % 8)) & 0x01;
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

struct define_huffman_table *find_DHT_by_type_and_id(struct context *ctx, int ac_dc_type, int table_id)
{
    for (int i = 0; i < ctx->count_DHTs; ++i)
    {
        if (ctx->DHTs[i].ac_dc_type == ac_dc_type && ctx->DHTs[i].table_id == table_id)
        {
            return &ctx->DHTs[i];
        }
    }

    return NULL;
}

// 根据color_id(YCbCr)找到相应的直流霍夫曼表和交流霍夫曼表
void find_DHT_by_color_id(struct context *ctx, int color_id, struct define_huffman_table **dc_dht, struct define_huffman_table **ac_dht)
{
    for (int i = 0; i < ctx->SOS.color_channel_count; ++i)
    {
        struct start_of_scan_channel_info *sos = &ctx->SOS.channel_info[i];
        if (sos->color_id == color_id)
        {
            *dc_dht = find_DHT_by_type_and_id(ctx, 0, sos->dc_dht_id); // 直流是0
            *ac_dht = find_DHT_by_type_and_id(ctx, 1, sos->ac_dht_id); // 交流是1
        }
    }
}

int calculate_coefficient_vli(uint8_t value, uint8_t mask)
{
    int value_bit_count = 0;
    uint8_t tmp = mask;
    while (tmp)
    {
        tmp &= (tmp - 1);
        value_bit_count++;
    }

    int coeff = (int)value;
    if (value >> (value_bit_count - 1) == 0)
        coeff = -((!value) & mask);

    return coeff;
}

void read_block(struct context *ctx, int color_id)
{
    int values[64] = {0};
    int count_values = 0;
    struct define_huffman_table *dc_dht = NULL, *ac_dht = NULL;
    find_DHT_by_color_id(ctx, color_id, &dc_dht, &ac_dht); // Y是1
    // [TODO] check pointer

    int found_dc = 0;                          // 标识是否处理了第一个数（直流分量），这会影响使用哪个DHT
    int found_0x00 = 0;                        // 标识是否遇到了0x00，如果遇到，说明该block解析结束
    struct define_huffman_table *dht = dc_dht; // 首先查找直流分量
    while (count_values < 64)                  // 一共就64个数
    {
        if (found_dc)
            dht = ac_dht; // 找到了直流分量尝试查找交流分量

        uint16_t test_code = 0, test_mask = 0; // 要与霍夫曼表匹配的码字和mask
        uint8_t test_value = 0;                // 霍夫曼表中该码字对应的值
        int found = 0;                         // 标识查找到当前码字
        for (int i = 0; i < 16; ++i)           // 一共最长就16位
        {
            test_code <<= 1;
            test_code |= get_bit(ctx);
            test_mask <<= 1;
            test_mask |= 0x01;
            // printf("test_code: %x\n", test_code);
            for (int j = 0; j < dht->leave_count_total; ++j) // 遍历dht表
            {
                struct define_huffman_table_code_item *item = &dht->items[j];
                if (test_mask == item->mask && test_code == item->code) // 位数相同（mask）并且code相同为找到
                {
                    found = 1;
                    test_value = item->value;
                    // printf("code: %x, value: %x, mask: %x\n", item->code, item->value, item->mask);
                    break;
                }
            }

            if (found) // 与霍夫曼表成功匹配，解析值
            {
                if (!found_dc) // 如果之前没有dc，那么这个是dc，没有后续解析
                {
                    found_dc = 1;
                    values[count_values++] =calculate_coefficient_vli(test_value, test_mask);
                }
                else // 处理ac，稍微复杂
                {
                    if (test_value == 0x00) // 如果找到0x00，后面全0，可以结束
                    {
                        found_0x00 = 1;
                        break;
                    }

                    uint8_t next_zero_count = (test_value >> 4) & 0x0F; // 高4位为接下来有几个0
                    for (int k = 0; k < next_zero_count; ++k)
                    {
                        values[count_values++] = 0;
                    }

                    uint8_t next_value_bit_count = (test_value >> 0) & 0x0F; // 低4位为接下来的数需要读几个bit
                    if (next_value_bit_count > 0)
                    {
                        uint16_t next_value = 0, next_mask = 0; // 读取指定bit出来的码字，需要使用VLI表进行解码
                        for (int k = 0; k < next_value_bit_count; ++k)
                        {
                            next_value <<= 1;
                            next_value |= get_bit(ctx);
                            next_mask <<= 1;
                            next_mask |= 0x01;
                        }

                        values[count_values++] = calculate_coefficient_vli(next_value, next_mask);
                    }
                }
                found = 0;
                break;
            }
        }

        if (found_0x00)
        {
            log_("found 0x00, finish, total: %d\n", count_values);
            break;
        }
    }

    for (int i = 0; i < 64; ++i)
    {
        printf("%d ", values[i]);
    }
    printf("\n");
}

void read_MCU(struct context *ctx)
{
    int horizontal_Y_block_count = ctx->SOF0.channel_info[0].horizontal_sample_rate;
    int vertical_Y_block_count = ctx->SOF0.channel_info[0].vertical_sample_rate;
    int horizontal_Cb_block_count = ctx->SOF0.channel_info[1].horizontal_sample_rate;
    int vertical_Cb_block_count = ctx->SOF0.channel_info[1].vertical_sample_rate;
    int horizontal_Cr_block_count = ctx->SOF0.channel_info[2].horizontal_sample_rate;
    int vertical_Cr_block_count = ctx->SOF0.channel_info[2].vertical_sample_rate;

    for (int i = 0; i < vertical_Y_block_count; ++i)
    {
        for (int j = 0; j < horizontal_Y_block_count; ++j)
        {
            read_block(ctx, 1);
        }
    }
    for (int i = 0; i < vertical_Cb_block_count; ++i)
    {
        for (int j = 0; j < horizontal_Cb_block_count; ++j)
        {
            read_block(ctx, 2);
        }
    }
    for (int i = 0; i < vertical_Cr_block_count; ++i)
    {
        for (int j = 0; j < horizontal_Cr_block_count; ++j)
        {
            read_block(ctx, 3);
        }
    }
}

void read_compressed_data(struct context *ctx)
{
    int horizontal_MCU_count = ctx->SOF0.width / ctx->SOF0.channel_info[0].horizontal_sample_rate;
    int vertical_MCU_count = ctx->SOF0.height / ctx->SOF0.channel_info[0].vertical_sample_rate;

    for (int i = 0; i < vertical_MCU_count; ++i)
    {
        for (int j = 0; j < horizontal_MCU_count; ++j)
        {
            read_MCU(ctx);
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

    // dump_DQTs(ctx);
    // dump_DHTs(ctx);
    // dump_SOF0(ctx);
    // dump_SOS(ctx);

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