#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "hardware/i2c.h"
#include "hardware/rtc.h"

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/binary_info.h"

#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"

#include "lib/FatFs_SPI/buzzer.h"
#include "lib/FatFs_SPI/ssd1306.h"
#include "lib/FatFs_SPI/rgb.h"
#include "lib/FatFs_SPI/matriz.h"

#define I2C_PORT_DISPLAY i2c1 // I2C1
#define I2C_SDA_DISPLAY 14    // GPIO14 - SDA
#define I2C_SCL_DISPLAY 15    // GPIO15 - SCL
#define endereco_DISPLAY 0x3C // Endereço do display SSD1306

#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
static int addr = 0x68;

#define botaoA 5
#define botaoB 6

volatile bool botaoA_pressionado = false;
volatile bool botaoB_pressionado = false;

absolute_time_t ultimo_tempo_botaoA;
absolute_time_t ultimo_tempo_botaoB;

const uint DEBOUNCE_MS = 200;

int16_t aceleracao[3], gyro[3], temp;

static void mpu6050_reset()
{
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(10);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x43;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x41;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

void gpio_callback(uint gpio, uint32_t events)
{
    absolute_time_t agora = get_absolute_time();

    if (gpio == botaoA && absolute_time_diff_us(ultimo_tempo_botaoA, agora) > DEBOUNCE_MS * 1000)
    {
        botaoA_pressionado = true;
        ultimo_tempo_botaoA = agora;
    }
    else if (gpio == botaoB && absolute_time_diff_us(ultimo_tempo_botaoB, agora) > DEBOUNCE_MS * 1000)
    {
        botaoB_pressionado = true;
        ultimo_tempo_botaoB = agora;
    }
}

static bool logger_enabled;
static const uint32_t period = 500;
static absolute_time_t next_log_time;

static char filename[20] = "MPU6050_data1.csv";

static sd_card_t *sd_get_by_name(const char *const name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

static FATFS *sd_get_fs_by_name(const char *name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

static void run_setrtc()
{
    const char *dateStr = strtok(NULL, " ");
    if (!dateStr)
    {
        printf("Missing argument\n");
        return;
    }
    int date = atoi(dateStr);

    const char *monthStr = strtok(NULL, " ");
    if (!monthStr)
    {
        printf("Missing argument\n");
        return;
    }
    int month = atoi(monthStr);

    const char *yearStr = strtok(NULL, " ");
    if (!yearStr)
    {
        printf("Missing argument\n");
        return;
    }
    int year = atoi(yearStr) + 2000;

    const char *hourStr = strtok(NULL, " ");
    if (!hourStr)
    {
        printf("Missing argument\n");
        return;
    }
    int hour = atoi(hourStr);

    const char *minStr = strtok(NULL, " ");
    if (!minStr)
    {
        printf("Missing argument\n");
        return;
    }
    int min = atoi(minStr);

    const char *secStr = strtok(NULL, " ");
    if (!secStr)
    {
        printf("Missing argument\n");
        return;
    }
    int sec = atoi(secStr);

    datetime_t t = {
        .year = (int16_t)year,
        .month = (int8_t)month,
        .day = (int8_t)date,
        .dotw = 0, // 0 is Sunday
        .hour = (int8_t)hour,
        .min = (int8_t)min,
        .sec = (int8_t)sec};
    rtc_set_datetime(&t);
}

static void run_format()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("amarelo");
        return;
    }

    rgb_set_color("azul");
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);

    if (FR_OK != fr)
    {
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("verde");
        return;
    }
    
    blinking_rgb(25, 50, "azul");
    rgb_set_color("verde");
}

static void run_mount()
{
    const char *arg1 = strtok(NULL, " ");

    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);

    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("amarelo");
        return;
    }

    FRESULT fr = f_mount(p_fs, arg1, 1);

    if (FR_OK != fr)
    {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("amarelo");
        return;
    }

    rgb_set_color("amarelo");

    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
    printf("SD 100%% montado\n");

    rgb_set_color("verde");
    buzzer_beep(4000, 50, 2);
}

static void run_unmount()
{
    const char *arg1 = strtok(NULL, " ");

    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;

    FATFS *p_fs = sd_get_fs_by_name(arg1);

    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        blinking_rgb(25, 100, "magenta");
        rgb_set_color("amarelo");
        return;
    }

    FRESULT fr = f_unmount(arg1);

    if (FR_OK != fr)
    {
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("verde");
        return;
    }

    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // in case medium is removed
    printf("SD ( %s ) desmontado\n", pSD->pcName);
    printf("SD 100%% montado\n");

    rgb_set_color("amarelo");
    buzzer_beep(4000, 50, 4);
}

static void run_getfree()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    DWORD fre_clust, fre_sect, tot_sect;
    FATFS *p_fs = sd_get_fs_by_name(arg1);

    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("amarelo");
        return;
    }

    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);

    if (FR_OK != fr)
    {
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("amarelo");
        return;
    }

    rgb_set_color("azul");
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;

    rgb_set_color("verde");

    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
}

static void run_ls()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = "";
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;
    if (arg1[0])
    {
        p_dir = arg1;
    }
    else
    {
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr)
        {
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            blinking_rgb(25, 50, "magenta");
            rgb_set_color("amarelo");
            return;
        }
        p_dir = cwdbuf;
    }

    printf("Directory Listing: %s\n\n", p_dir);
    DIR dj;
    FILINFO fno;
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, p_dir, "*");

    if (FR_OK != fr)
    {
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("amarelo");
        return;
    }

    while (fr == FR_OK && fno.fname[0])
    {
        rgb_set_color("azul");
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        if (fno.fattrib & AM_DIR)
            pcAttrib = pcDirectory;
        else if (fno.fattrib & AM_RDO)
            pcAttrib = pcReadOnlyFile;
        else
            pcAttrib = pcWritableFile;
        printf("%s [%s] [size=%llu]\n", fno.fname, pcAttrib, fno.fsize);

        fr = f_findnext(&dj, &fno);
    }

    f_closedir(&dj);

    blinking_rgb(25, 50, "azul");
    rgb_set_color("verde");
}

static void run_cat()
{
    char *arg1 = strtok(NULL, " ");
    if (!arg1)
    {
        printf("Missing argument\n");
        return;
    }
    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);
    if (FR_OK != fr)
    {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil))
    {
        printf("%s", buf);
    }
    fr = f_close(&fil);
    if (FR_OK != fr)
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
}

// Função para capturar dados e salvar no arquivo *.txt
void capture_data()
{
    rgb_set_color("vermelho");
    buzzer_beep(6000, 150, 1);

    printf("\nCapturando dados do MPU6050. Aguarde finalização...\n");
    FIL file;
    FRESULT res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);

    if (res != FR_OK)
    {
        printf("\n[ERRO] Não foi possível abrir o arquivo para escrita. Monte o Cartao.\n");
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("amarelo");
        return;
    }

    const char *header = "numero_amostra,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\n";
    UINT bw;
    f_write(&file, header, strlen(header), &bw);

    for (int i = 0; i < 128; i++)
    {
        mpu6050_read_raw(aceleracao, gyro, &temp);

        float ax = aceleracao[0] / 16384.0f;
        float ay = aceleracao[1] / 16384.0f;
        float az = aceleracao[2] / 16384.0f;

        float roll = atan2(ay, az) * 180.0f / M_PI;
        float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
        static float yaw = 0.0f; // mantenha acumulado

        static uint64_t tempo_anterior = 0;
        if (tempo_anterior == 0)
            tempo_anterior = to_us_since_boot(get_absolute_time());

        float gz = gyro[2] / 131.0f; // sensibilidade padrão do MPU6050 em °/s

        uint64_t tempo_atual = to_us_since_boot(get_absolute_time());
        float dt = (tempo_atual - tempo_anterior) / 1000000.0f;
        tempo_anterior = tempo_atual;

        yaw += gz * dt;

        if (yaw > 180.0f)
            yaw -= 360.0f;
        if (yaw < -180.0f)
            yaw += 360.0f;

        char buffer[80];
        sprintf(buffer, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", i + 1, ax, ay, az, roll, pitch, yaw);

        UINT bw;
        res = f_write(&file, buffer, strlen(buffer), &bw);
        if (res != FR_OK)
        {
            printf("[ERRO] Não foi possível escrever no arquivo. Monte o Cartao.\n");
            blinking_rgb(25, 50, "magenta");
            rgb_set_color("amarelo");
            f_close(&file);
            return;
        }
        sleep_ms(50);
    }

    rgb_set_color("verde");
    buzzer_beep(6000, 150, 2);

    f_close(&file);
    printf("\nDados salvos no arquivo %s.\n\n", filename);
}

// Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename)
{
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK)

    {
        printf("[ERRO] Não foi possível abrir o arquivo para leitura. Verifique se o Cartão está montado ou se o arquivo existe.\n\n");
        blinking_rgb(25, 50, "magenta");
        rgb_set_color("amarelo");
        return;
    }

    char buffer[128];
    UINT br;
    printf("Conteúdo do arquivo %s:\n", filename);

    while (f_read(&file, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0)
    {
        rgb_set_color("azul");
        buffer[br] = '\0';
        printf("%s", buffer);
    }

    blinking_rgb(25, 50, "azul");
    rgb_set_color("verde");

    f_close(&file);
    
    printf("\nLeitura do arquivo %s concluída.\n\n", filename);
}

static void run_help()
{
    printf("\nComandos disponíveis:\n\n");
    printf("Digite 'a' para montar o cartão SD\n");
    printf("Digite 'b' para desmontar o cartão SD\n");
    printf("Digite 'c' para listar arquivos\n");
    printf("Digite 'd' para mostrar conteúdo do arquivo\n");
    printf("Digite 'e' para obter espaço livre no cartão SD\n");
    printf("Digite 'f' para capturar dados do MPU6050 e salvar no arquivo\n");
    printf("Digite 'g' para formatar o cartão SD\n");
    printf("Digite 'h' para exibir os comandos disponíveis\n");
    printf("\nEscolha o comando:  ");
}

typedef void (*p_fn_t)();
typedef struct
{
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <YY> <hh> <mm> <ss>: Set Real Time Clock"},
    {"format", run_format, "format [<drive#:>]: Formata o cartão SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cartão SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cartão SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Espaço livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <filename>: Mostra conteúdo do arquivo"},
    {"help", run_help, "help: Mostra comandos disponíveis"},
};

static void process_stdio(int cRxedChar)
{
    static char cmd[256];
    static size_t ix;

    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != (char)127)
        return;
    printf("%c", cRxedChar); // echo
    stdio_flush();
    if (cRxedChar == '\r')
    {
        printf("%c", '\n');
        stdio_flush();

        if (!strnlen(cmd, sizeof cmd))
        {
            printf("> ");
            stdio_flush();
            return;
        }
        char *cmdn = strtok(cmd, " ");
        if (cmdn)
        {
            size_t i;
            for (i = 0; i < count_of(cmds); ++i)
            {
                if (0 == strcmp(cmds[i].command, cmdn))
                {
                    (*cmds[i].function)();
                    break;
                }
            }
            if (count_of(cmds) == i)
                printf("Command \"%s\" not found\n", cmdn);
        }
        ix = 0;
        memset(cmd, 0, sizeof cmd);
        printf("\n> ");
        stdio_flush();
    }
    else
    {
        if (cRxedChar == '\b' || cRxedChar == (char)127)
        {
            if (ix > 0)
            {
                ix--;
                cmd[ix] = '\0';
            }
        }
        else
        {
            if (ix < sizeof cmd - 1)
            {
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}

void gpio_callback_botaoA(uint gpio, uint32_t events)
{
    absolute_time_t agora = get_absolute_time();
    if (absolute_time_diff_us(ultimo_tempo_botaoA, agora) > DEBOUNCE_MS * 1000)
    {
        botaoA_pressionado = true;
        ultimo_tempo_botaoA = agora;
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(5000);

    time_init();
    rgb_init();

    // Inicialização dos botões
    gpio_init(botaoA);
    gpio_set_dir(botaoA, GPIO_IN);
    gpio_pull_up(botaoA);

    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);

    // Interrupção com callback compartilhado
    gpio_set_irq_enabled_with_callback(botaoA, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(botaoB, GPIO_IRQ_EDGE_FALL, true);

    rgb_set_color("amarelo");

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
    mpu6050_reset();

    i2c_init(I2C_PORT_DISPLAY, 400 * 1000); // I2C Initialisation. Using it at 400Khz.

    gpio_set_function(I2C_SDA_DISPLAY, GPIO_FUNC_I2C);                            // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL_DISPLAY, GPIO_FUNC_I2C);                            // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA_DISPLAY);                                                // Pull up the data line
    gpio_pull_up(I2C_SCL_DISPLAY);                                                // Pull up the clock line
    ssd1306_t ssd;                                                                // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco_DISPLAY, I2C_PORT_DISPLAY); // Inicializa o display
    ssd1306_config(&ssd);                                                         // Configura o display
    ssd1306_send_data(&ssd);                                                      // Envia os dados para o display

    ssd1306_fill(&ssd, false); // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_send_data(&ssd);   // Envia os dados para o display

    bool cor = true;

    ssd1306_fill(&ssd, !cor);                     // Limpa o display
    ssd1306_draw_string(&ssd, "Sistema", 2, 28);  // Desenha uma string
    ssd1306_draw_string(&ssd, "Iniciado", 2, 37); // Desenha uma string
    ssd1306_send_data(&ssd);

    printf("FatFS SPI example\n");
    printf("\033[2J\033[H"); // Limpa tela
    printf("\n> ");
    stdio_flush();

    run_help();
    while (true)
    {
        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar)
            process_stdio(cRxedChar);

        if (cRxedChar == 'a') // Monta o SD card se pressionar 'a'
        {
            printf("\nMontando o SD...\n");

            ssd1306_fill(&ssd, !cor);                     // Limpa o display
            ssd1306_draw_string(&ssd, "Montando", 2, 28); // Desenha uma string
            ssd1306_draw_string(&ssd, "SD...", 2, 37);    // Desenha uma string
            ssd1306_send_data(&ssd);

            run_mount();

            ssd1306_fill(&ssd, !cor);                       // Limpa o display
            ssd1306_draw_string(&ssd, "SD Montado", 2, 28); // Desenha uma string
            ssd1306_send_data(&ssd);

            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'b') // Desmonta o SD card se pressionar 'b'
        {
            printf("\nDesmontando o SD. Aguarde...\n");

            ssd1306_fill(&ssd, !cor);                        // Limpa o display
            ssd1306_draw_string(&ssd, "Desmontando", 2, 28); // Desenha uma string
            ssd1306_draw_string(&ssd, "SD...", 2, 37);       // Desenha uma string
            ssd1306_send_data(&ssd);

            run_unmount();

            ssd1306_fill(&ssd, !cor);                          // Limpa o display
            ssd1306_draw_string(&ssd, "SD Desmontado", 2, 28); // Desenha uma string
            ssd1306_send_data(&ssd);

            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'c') // Lista diretórios e os arquivos se pressionar 'c'
        {
            printf("\nListagem de arquivos no cartão SD.\n");

            ssd1306_fill(&ssd, !cor);                        // Limpa o display
            ssd1306_draw_string(&ssd, "Listando", 2, 28);    // Desenha uma string
            ssd1306_draw_string(&ssd, "Arquivos...", 2, 37); // Desenha uma string
            ssd1306_send_data(&ssd);

            run_ls();

            ssd1306_fill(&ssd, !cor);                            // Limpa o display
            ssd1306_draw_string(&ssd, "Lista Concluida", 2, 28); // Desenha uma string
            ssd1306_send_data(&ssd);

            printf("\nListagem concluída.\n");

            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'd') // Exibe o conteúdo do arquivo se pressionar 'd'
        {
            ssd1306_fill(&ssd, !cor);                       // Limpa o display
            ssd1306_draw_string(&ssd, "Lendo", 2, 28);      // Desenha uma string
            ssd1306_draw_string(&ssd, "Arquivo...", 2, 37); // Desenha uma string
            ssd1306_send_data(&ssd);

            read_file(filename);

            ssd1306_fill(&ssd, !cor);                         // Limpa o display
            ssd1306_draw_string(&ssd, "Arquivo Lido", 2, 28); // Desenha uma string
            ssd1306_send_data(&ssd);

            printf("Escolha o comando (h = help):  ");
        }
        if (cRxedChar == 'e') // Obtém o espaço livre no SD card se pressionar 'e'
        {
            ssd1306_fill(&ssd, !cor); // Limpa o display
            ssd1306_send_data(&ssd);

            printf("\nObtendo espaço livre no SD.\n\n");

            ssd1306_fill(&ssd, !cor);                           // Limpa o display
            ssd1306_draw_string(&ssd, "Obtendo Espaco", 2, 28); // Desenha uma string
            ssd1306_draw_string(&ssd, "Livre...", 2, 37);       // Desenha uma string
            ssd1306_send_data(&ssd);

            run_getfree();

            ssd1306_fill(&ssd, !cor);                          // Limpa o display
            ssd1306_draw_string(&ssd, "Espaco Obtido", 2, 28); // Desenha uma string
            ssd1306_send_data(&ssd);

            printf("\nEspaço livre obtido.\n");

            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'f') // Captura dados do MPU6050 e salva no arquivo se pressionar 'f'
        {
            ssd1306_fill(&ssd, !cor);                       // Limpa o display
            ssd1306_draw_string(&ssd, "Capturando", 2, 28); // Desenha uma string
            ssd1306_draw_string(&ssd, "Dados...", 2, 37);   // Desenha uma string
            ssd1306_send_data(&ssd);

            capture_data();

            ssd1306_fill(&ssd, !cor);                          // Limpa o display
            ssd1306_draw_string(&ssd, "Dados Obtidos", 2, 28); // Desenha uma string
            ssd1306_send_data(&ssd);

            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'g') // Formata o SD card se pressionar 'g'
        {
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");

            ssd1306_fill(&ssd, !cor);                        // Limpa o display
            ssd1306_draw_string(&ssd, "Formatacao", 2, 28);  // Desenha uma string
            ssd1306_draw_string(&ssd, "Iniciada...", 2, 37); // Desenha uma string
            ssd1306_send_data(&ssd);

            run_format();

            ssd1306_fill(&ssd, !cor);                       // Limpa o display
            ssd1306_draw_string(&ssd, "Formatacao", 2, 28); // Desenha uma string
            ssd1306_draw_string(&ssd, "Concluida", 2, 37);  // Desenha uma string
            ssd1306_send_data(&ssd);

            printf("\nFormatação concluída.\n\n");

            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'h') // Exibe os comandos disponíveis se pressionar 'h'
        {
            ssd1306_fill(&ssd, !cor);                       // Limpa o display
            ssd1306_draw_string(&ssd, "Ajuda", 2, 28);      // Desenha uma string
            ssd1306_draw_string(&ssd, "Solicitada", 2, 37); // Desenha uma string
            ssd1306_send_data(&ssd);

            run_help();
        }
        if (botaoA_pressionado)
        {
            botaoA_pressionado = false;

            ssd1306_fill(&ssd, !cor);
            ssd1306_draw_string(&ssd, "Capturando", 2, 28);
            ssd1306_draw_string(&ssd, "Dados (BTN)...", 2, 37);
            ssd1306_send_data(&ssd);

            capture_data();

            ssd1306_fill(&ssd, !cor);
            ssd1306_draw_string(&ssd, "Dados OK", 2, 28);
            ssd1306_draw_string(&ssd, "por Botao A", 2, 37);
            ssd1306_send_data(&ssd);

            printf("[INFO] Dados capturados via Botão A (GPIO 5).\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (botaoB_pressionado)
        {
            botaoB_pressionado = false;
            reset_usb_boot(0, 0);
        }
        sleep_ms(500);
    }
    return 0;
}