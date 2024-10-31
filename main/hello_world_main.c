#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <driver/touch_pad.h>
#include <stdio.h>
#include "driver/timer.h"  // Biblioteca do GPTimer
#include "esp_timer.h"

#define NUM_AMOSTRAS 20  // Coletar 20 amostras para cada tarefa
#define M_FIRME 10       // (m, k)-firme, 10 ativações de janela para cada tarefa
#define K_TOTAL 20       // Total de ativações consideradas para análise firme

// Definição dos pinos dos sensores e atuadores
#define INJECION_SENSOR_PIN GPIO_NUM_18
#define TEMPERATURA_SENSOR_PIN GPIO_NUM_19
#define ABS_SENSOR_PIN GPIO_NUM_21
#define AIRBAG_SENSOR_PIN GPIO_NUM_22
#define CINTO_SENSOR_PIN GPIO_NUM_23

// Definição dos touchpads da ESP32
#define TOUCH_PAD_SENSOR_1 TOUCH_PAD_NUM8
#define TOUCH_PAD_SENSOR_2 TOUCH_PAD_NUM9

// Variáveis globais para armazenar o estado dos sensores, tempos de execução, WCET, WCRT e contagem de deadlines
int injecao_value = 0;
int temperatura_value = 0;
int abs_value = 0;
int airbag_value = 0;
int cinto_value = 1;

int64_t injecao_exec_time = 0;
int64_t injecao_response_time = 0;
int64_t temperatura_exec_time = 0;
int64_t temperatura_response_time = 0;
int64_t abs_exec_time = 0;
int64_t abs_response_time = 0;
int64_t airbag_exec_time = 0;
int64_t airbag_response_time = 0;
int64_t cinto_exec_time = 0;
int64_t cinto_response_time = 0;

int64_t injecao_wcet = 0;
int64_t temperatura_wcet = 0;
int64_t abs_wcet = 0;
int64_t airbag_wcet = 0;
int64_t cinto_wcet = 0;

int64_t injecao_wcrt = 0;
int64_t temperatura_wcrt = 0;
int64_t abs_wcrt = 0;
int64_t airbag_wcrt = 0;
int64_t cinto_wcrt = 0;

int injecao_deadline_miss = 0;
int temperatura_deadline_miss = 0;
int abs_deadline_miss = 0;
int airbag_deadline_miss = 0;
int cinto_deadline_miss = 0;

int injecao_hwm = 0;
int abs_hwm = 0;
int airbag_hwm = 0;
int cinto_hwm = 0;
int temperatura_hwm = 0;

SemaphoreHandle_t mutex; // Declaração do mutex

volatile int simulate_delay = 0;  // Variável para controlar a simulação de atrasos

// Função de callback para interrupção do sensor touch
void touch_pad_isr_handler(void *arg) {
    int sensor_num = (int)arg;
    printf("Interrupção de Sensor Touch: %d\n", sensor_num);

    // Alternar o estado de simulate_delay
    simulate_delay = !simulate_delay;

    printf("Simulação de atraso %s.\n", simulate_delay ? "ativada" : "desativada");
}

// Função para controlar os atuadores
void execute_actuator(int *actuator_value, int new_value) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    *actuator_value = new_value;
    xSemaphoreGive(mutex);
}

// Task para controlar a injeção eletrônica e calcular WCET, WCRT e HWM
void vTaskInjecaoEletronica(void *pvParameters) {
    int amostras = 0;
    const TickType_t xPeriod = pdMS_TO_TICKS(500);  // Período de 500 ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // Aguarda até o próximo período
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        int new_value = gpio_get_level(INJECION_SENSOR_PIN);  // Leitura do sensor
        execute_actuator(&injecao_value, new_value);

        if (injecao_value == 1) {
            uint64_t release_time = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &release_time);  // Tempo de liberação da tarefa

            uint64_t beginEL = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &beginEL);  // Tempo de início de execução

            // Adicionando carga de trabalho artificial
            volatile int dummy = 0;
            for (int i = 0; i < 10000; i++) {
                dummy += i;
            }

            // Inserir atraso adicional se simulate_delay estiver ativado
            if (simulate_delay) {
                vTaskDelay(pdMS_TO_TICKS(1));  // Atraso de 1 ms
            }

            uint64_t endTimeEL = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &endTimeEL);  // Tempo de término de execução
            injecao_exec_time = (endTimeEL - beginEL);  // Tempo de execução
            injecao_response_time = (endTimeEL - release_time);  // Tempo de resposta

            xSemaphoreTake(mutex, portMAX_DELAY);

            // Atualiza o WCET
            if (injecao_exec_time > injecao_wcet) {
                injecao_wcet = injecao_exec_time;
            }

            // Atualiza o WCRT
            if (injecao_response_time > injecao_wcrt) {
                injecao_wcrt = injecao_response_time;
            }

            // Atualiza o HWM
            if (injecao_exec_time > injecao_hwm) {
                injecao_hwm = injecao_exec_time;
            }

            // (m, k)-firme: Verifica se a tarefa perdeu o deadline com base no tempo de execução
            if (injecao_exec_time > 500) {  // Deadline de 500 us
                injecao_deadline_miss++;
                printf("Injeção Eletrônica - Deadline perdido. Tempo de execução: %lld us\n", injecao_exec_time);
            }

            xSemaphoreGive(mutex);
        } else {
            injecao_exec_time = 0;
            injecao_response_time = 0;  // Redefine o response_time para zero
        }

        amostras++;
        if (amostras >= K_TOTAL) {
            xSemaphoreTake(mutex, portMAX_DELAY);
            int skip_factor = injecao_deadline_miss;
            if (skip_factor > (K_TOTAL - M_FIRME)) {
                printf("Injeção Eletrônica - Fator Skip: %d\n", skip_factor);
            }
            injecao_deadline_miss = 0;
            amostras = 0;
            xSemaphoreGive(mutex);
        }
    }
}

// Task para monitorar a temperatura do motor e calcular WCET, WCRT e HWM
void vTaskMonitorarTemperatura(void *pvParameters) {
    int amostras = 0;
    const TickType_t xPeriod = pdMS_TO_TICKS(20);  // Período de 20 ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // Aguarda até o próximo período
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        int new_value = gpio_get_level(TEMPERATURA_SENSOR_PIN);  // Leitura do sensor
        execute_actuator(&temperatura_value, new_value);

        if (temperatura_value == 1) {
            uint64_t release_time = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &release_time);

            uint64_t beginIT = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &beginIT);  // Tempo de início de execução

            // Adicionando carga de trabalho artificial
            volatile int dummy = 0;
            for (int i = 0; i < 5000; i++) {
                dummy += i;
            }

            // Inserir atraso adicional se simulate_delay estiver ativado
            if (simulate_delay) {
                vTaskDelay(pdMS_TO_TICKS(5));  // Atraso de 5 ms
            }

            uint64_t endTimeIT = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &endTimeIT);  // Tempo de término de execução
            temperatura_exec_time = (endTimeIT - beginIT);  // Tempo de execução
            temperatura_response_time = (endTimeIT - release_time);  // Tempo de resposta

            xSemaphoreTake(mutex, portMAX_DELAY);

            // Atualiza o WCET
            if (temperatura_exec_time > temperatura_wcet) {
                temperatura_wcet = temperatura_exec_time;
            }

            // Atualiza o WCRT
            if (temperatura_response_time > temperatura_wcrt) {
                temperatura_wcrt = temperatura_response_time;
            }

            // Atualiza o HWM
            if (temperatura_exec_time > temperatura_hwm) {
                temperatura_hwm = temperatura_exec_time;
            }

            // (m, k)-firme: Verifica se a tarefa perdeu o deadline com base no tempo de execução
            if (temperatura_exec_time > 20000) {  // Deadline de 20 ms = 20000 us
                temperatura_deadline_miss++;
                printf("Temperatura - Deadline perdido. Tempo de execução: %lld us\n", temperatura_exec_time);
            }

            xSemaphoreGive(mutex);
        } else {
            temperatura_exec_time = 0;
            temperatura_response_time = 0;  // Redefine o response_time para zero
        }

        amostras++;
        if (amostras >= K_TOTAL) {
            xSemaphoreTake(mutex, portMAX_DELAY);
            int skip_factor = temperatura_deadline_miss;
            if (skip_factor > (K_TOTAL - M_FIRME)) {
                printf("Temperatura - Fator Skip: %d\n", skip_factor);
            }
            temperatura_deadline_miss = 0;
            amostras = 0;
            xSemaphoreGive(mutex);
        }
    }
}

// Task para monitorar e controlar o sistema ABS e calcular WCET, WCRT e HWM
void vTaskControleABS(void *pvParameters) {
    int amostras = 0;
    const TickType_t xPeriod = pdMS_TO_TICKS(100);  // Período de 100 ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // Aguarda até o próximo período
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        int new_value = gpio_get_level(ABS_SENSOR_PIN);  // Leitura do sensor
        execute_actuator(&abs_value, new_value);

        if (abs_value == 1) {
            uint64_t release_time = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &release_time);

            uint64_t beginABS = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &beginABS);  // Tempo de início de execução

            // Adicionando carga de trabalho artificial
            volatile int dummy = 0;
            for (int i = 0; i < 8000; i++) {
                dummy += i;
            }

            // Inserir atraso adicional se simulate_delay estiver ativado
            if (simulate_delay) {
                vTaskDelay(pdMS_TO_TICKS(10));  // Atraso de 10 ms
            }

            uint64_t endTimeABS = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &endTimeABS);  // Tempo de término de execução
            abs_exec_time = (endTimeABS - beginABS);  // Tempo de execução
            abs_response_time = (endTimeABS - release_time);  // Tempo de resposta

            xSemaphoreTake(mutex, portMAX_DELAY);

            // Atualiza o WCET
            if (abs_exec_time > abs_wcet) {
                abs_wcet = abs_exec_time;
            }

            // Atualiza o WCRT
            if (abs_response_time > abs_wcrt) {
                abs_wcrt = abs_response_time;
            }

            // Atualiza o HWM
            if (abs_exec_time > abs_hwm) {
                abs_hwm = abs_exec_time;
            }

            // (m, k)-firme: Verifica se a tarefa perdeu o deadline com base no tempo de execução
            if (abs_exec_time > 100000) {  // Deadline de 100 ms = 100000 us
                abs_deadline_miss++;
                printf("ABS - Deadline perdido. Tempo de execução: %lld us\n", abs_exec_time);
            }

            xSemaphoreGive(mutex);
        } else {
            abs_exec_time = 0;
            abs_response_time = 0;  // Redefine o response_time para zero
        }

        amostras++;
        if (amostras >= K_TOTAL) {
            xSemaphoreTake(mutex, portMAX_DELAY);
            int skip_factor = abs_deadline_miss;
            if (skip_factor > (K_TOTAL - M_FIRME)) {
                printf("ABS - Fator Skip: %d\n", skip_factor);
            }
            abs_deadline_miss = 0;
            amostras = 0;
            xSemaphoreGive(mutex);
        }
    }
}

// Task para controlar o Airbag e calcular WCET, WCRT e HWM
void vTaskControleAirbag(void *pvParameters) {
    int amostras = 0;
    const TickType_t xPeriod = pdMS_TO_TICKS(100);  // Período de 100 ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // Aguarda até o próximo período
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        int new_value = gpio_get_level(AIRBAG_SENSOR_PIN);  // Leitura do sensor de airbag
        execute_actuator(&airbag_value, new_value);

        if (airbag_value == 1) {
            uint64_t release_time = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &release_time);

            uint64_t beginAIR = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &beginAIR);  // Tempo de início de execução

            // Adicionando carga de trabalho artificial
            volatile int dummy = 0;
            for (int i = 0; i < 8000; i++) {
                dummy += i;
            }

            // Inserir atraso adicional se simulate_delay estiver ativado
            if (simulate_delay) {
                vTaskDelay(pdMS_TO_TICKS(15));  // Atraso de 15 ms
            }

            uint64_t endTimeAIR = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &endTimeAIR);  // Tempo de término de execução
            airbag_exec_time = (endTimeAIR - beginAIR);  // Tempo de execução
            airbag_response_time = (endTimeAIR - release_time);  // Tempo de resposta

            xSemaphoreTake(mutex, portMAX_DELAY);

            // Atualiza o WCET
            if (airbag_exec_time > airbag_wcet) {
                airbag_wcet = airbag_exec_time;
            }

            // Atualiza o WCRT
            if (airbag_response_time > airbag_wcrt) {
                airbag_wcrt = airbag_response_time;
            }

            // Atualiza o HWM
            if (airbag_exec_time > airbag_hwm) {
                airbag_hwm = airbag_exec_time;
            }

            // (m, k)-firme: Verifica se a tarefa perdeu o deadline com base no tempo de execução
            if (airbag_exec_time > 100000) {  // Deadline de 100 ms = 100000 us
                airbag_deadline_miss++;
                printf("Airbag - Deadline perdido. Tempo de execução: %lld us\n", airbag_exec_time);
            }

            xSemaphoreGive(mutex);
        } else {
            airbag_exec_time = 0;
            airbag_response_time = 0;  // Redefine o response_time para zero
        }

        amostras++;
        if (amostras >= K_TOTAL) {
            xSemaphoreTake(mutex, portMAX_DELAY);
            int skip_factor = airbag_deadline_miss;
            if (skip_factor > (K_TOTAL - M_FIRME)) {
                printf("Airbag - Fator Skip: %d\n", skip_factor);
            }
            airbag_deadline_miss = 0;
            amostras = 0;
            xSemaphoreGive(mutex);
        }
    }
}

// Task para controlar o Cinto de Segurança e calcular WCET, WCRT e HWM
void vTaskControleCinto(void *pvParameters) {
    int amostras = 0;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);  // Período de 1 segundo
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // Aguarda até o próximo período
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        int new_value = gpio_get_level(CINTO_SENSOR_PIN);  // Leitura do sensor de cinto de segurança
        execute_actuator(&cinto_value, new_value);

        if (cinto_value == 1) {
            uint64_t release_time = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &release_time);

            uint64_t beginCINTO = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &beginCINTO);  // Tempo de início de execução

            // Adicionando carga de trabalho artificial
            volatile int dummy = 0;
            for (int i = 0; i < 5000; i++) {
                dummy += i;
            }

            // Inserir atraso adicional se simulate_delay estiver ativado
            if (simulate_delay) {
                vTaskDelay(pdMS_TO_TICKS(20));  // Atraso de 20 ms
            }

            uint64_t endTimeCINTO = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &endTimeCINTO);  // Tempo de término de execução
            cinto_exec_time = (endTimeCINTO - beginCINTO);  // Tempo de execução
            cinto_response_time = (endTimeCINTO - release_time);  // Tempo de resposta

            xSemaphoreTake(mutex, portMAX_DELAY);

            // Atualiza o WCET
            if (cinto_exec_time > cinto_wcet) {
                cinto_wcet = cinto_exec_time;
            }

            // Atualiza o WCRT
            if (cinto_response_time > cinto_wcrt) {
                cinto_wcrt = cinto_response_time;
            }

            // Atualiza o HWM
            if (cinto_exec_time > cinto_hwm) {
                cinto_hwm = cinto_exec_time;
            }

            // (m, k)-firme: Verifica se a tarefa perdeu o deadline com base no tempo de execução
            if (cinto_exec_time > 100000) {  // Deadline de 100 ms = 100000 us
                cinto_deadline_miss++;
                printf("Cinto - Deadline perdido. Tempo de execução: %lld us\n", cinto_exec_time);
            }

            xSemaphoreGive(mutex);
        } else {
            cinto_exec_time = 0;
            cinto_response_time = 0;  // Redefine o response_time para zero
        }

        amostras++;
        if (amostras >= K_TOTAL) {
            xSemaphoreTake(mutex, portMAX_DELAY);
            int skip_factor = cinto_deadline_miss;
            if (skip_factor > (K_TOTAL - M_FIRME)) {
                printf("Cinto - Fator Skip: %d\n", skip_factor);
            }
            cinto_deadline_miss = 0;
            amostras = 0;
            xSemaphoreGive(mutex);
        }
    }
}

// Task para exibir o display com WCET, WCRT, HWM e fator de skip de todas as tasks
void vTaskAtualizarDisplay(void *pvParameters) {
    while(1) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        printf("\n----- Atualização do Display -----\n");
        printf("Injeção Eletrônica: %d | tempo de execução: %lld us | WCRT: %lld us | Deadline: 500 us | WCET: %lld us | HWM: %d us\n",
               injecao_value, injecao_exec_time, injecao_wcrt, injecao_wcet, injecao_hwm);
        printf("Temperatura do Motor: %d | tempo de execução: %lld us | WCRT: %lld us | Deadline: 20 ms | WCET: %lld us | HWM: %d us\n",
               temperatura_value, temperatura_exec_time, temperatura_wcrt, temperatura_wcet, temperatura_hwm);
        printf("ABS: %d | tempo de execução: %lld us | WCRT: %lld us | Deadline: 100 ms | WCET: %lld us | HWM: %d us\n",
               abs_value, abs_exec_time, abs_wcrt, abs_wcet, abs_hwm);
        printf("Airbag: %d | tempo de execução: %lld us | WCRT: %lld us | Deadline: 100 ms | WCET: %lld us | HWM: %d us\n",
               airbag_value, airbag_exec_time, airbag_wcrt, airbag_wcet, airbag_hwm);
        printf("Cinto de Segurança: %s | tempo de execução: %lld us | WCRT: %lld us | Deadline: 100 ms | WCET: %lld us | HWM: %d us\n",
               cinto_value == 0 ? "Desafivelado" : "Afivelado", cinto_exec_time, cinto_wcrt, cinto_wcet, cinto_hwm);
        printf("-----------------------------------\n\n");
        xSemaphoreGive(mutex);

        vTaskDelay(pdMS_TO_TICKS(1000));  // Atualiza o display a cada 1 segundo
    }
}

// Configuração dos touchpads
void config_touch_pad() {
    esp_err_t err = touch_pad_init();
    if (err == ESP_OK) {
        touch_pad_config(TOUCH_PAD_SENSOR_1, 0);
        touch_pad_config(TOUCH_PAD_SENSOR_2, 0);

        // Registra a interrupção dos touchpads
        touch_pad_isr_register(touch_pad_isr_handler, (void *)TOUCH_PAD_SENSOR_1);
        touch_pad_isr_register(touch_pad_isr_handler, (void *)TOUCH_PAD_SENSOR_2);

        // Ativa a interrupção dos touchpads
        touch_pad_intr_enable();
    } else {
        printf("Erro ao inicializar o touch pad: %d\n", err);
    }
}

void app_main() {
    // Inicializar GPIOs (pinos dos sensores)
    esp_rom_gpio_pad_select_gpio(INJECION_SENSOR_PIN);
    gpio_set_direction(INJECION_SENSOR_PIN, GPIO_MODE_INPUT);

    esp_rom_gpio_pad_select_gpio(TEMPERATURA_SENSOR_PIN);
    gpio_set_direction(TEMPERATURA_SENSOR_PIN, GPIO_MODE_INPUT);

    esp_rom_gpio_pad_select_gpio(ABS_SENSOR_PIN);
    gpio_set_direction(ABS_SENSOR_PIN, GPIO_MODE_INPUT);

    esp_rom_gpio_pad_select_gpio(AIRBAG_SENSOR_PIN);
    gpio_set_direction(AIRBAG_SENSOR_PIN, GPIO_MODE_INPUT);

    esp_rom_gpio_pad_select_gpio(CINTO_SENSOR_PIN);
    gpio_set_direction(CINTO_SENSOR_PIN, GPIO_MODE_INPUT);

    // Configuração do General Purpose Timer
    timer_config_t timer_config = {
        .divider = 80,  // Prescaler: 80 MHz / 80 = 1 MHz (1 tick = 1 us)
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = false,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, TIMER_0);

    // Configura os touchpads
    config_touch_pad();

    // Cria o mutex
    mutex = xSemaphoreCreateMutex();

    // Criando tasks para cada subsistema com prioridades ajustadas
    xTaskCreate(vTaskInjecaoEletronica, "Injeção Eletrônica", 2048, NULL, 3, NULL);  // Soft deadline, prioridade média
    xTaskCreate(vTaskMonitorarTemperatura, "Monitorar Temperatura", 2048, NULL, 2, NULL);  // Soft deadline, prioridade média-baixa
    xTaskCreate(vTaskControleABS, "Controle ABS", 2048, NULL, 5, NULL);  // Hard deadline, prioridade alta
    xTaskCreate(vTaskControleAirbag, "Controle Airbag", 2048, NULL, 5, NULL);  // Hard deadline, prioridade alta
    xTaskCreate(vTaskControleCinto, "Controle Cinto", 2048, NULL, 5, NULL);  // Hard deadline, prioridade alta
    xTaskCreate(vTaskAtualizarDisplay, "Atualizar Display", 2048, NULL, 1, NULL);  // Prioridade baixa
}