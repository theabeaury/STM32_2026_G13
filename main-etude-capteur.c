/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - BE_2
  *                   Grove Ultrasonic Ranger (single-pin PB0) + AM2302 (PA1) + LCD I2C
  *
  * Ce fichier implémente la boucle principale du BE_2.
  * Il gère deux capteurs à protocole single-wire et un afficheur LCD I2C :
  *   - Grove Ultrasonic Ranger sur PB0 : mesure de distance par ultrasons
  *   - AM2302 (DHT22) sur PA1          : mesure de température et d'humidité
  *   - Écran LCD 16x2 via I2C1         : affichage des mesures en temps réel
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>   /* Pour snprintf() — formatage des chaînes d'affichage */
#include <string.h>  /* Pour les fonctions de manipulation de chaînes (non utilisé directement ici) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Adresse I2C du LCD décalée d'un bit à gauche (format attendu par HAL : 7 bits + 0 en LSB) */
#define LCD_I2C_ADDR  (0x3E << 1)

/* Grove Ultrasonic Ranger — broche unique PB0 (Trigger et Echo partagés) */
#define ULTRA_PIN     GPIO_PIN_0   /* Broche GPIO du capteur ultrasonique */
#define ULTRA_PORT    GPIOB        /* Port GPIO B */

/* AM2302 (DHT22) — protocole single-wire sur PA1 */
#define DHT_PIN       GPIO_PIN_1   /* Broche GPIO du capteur de température/humidité */
#define DHT_PORT      GPIOA        /* Port GPIO A */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Handle du périphérique I2C1 — utilisé pour la communication avec le LCD */
I2C_HandleTypeDef hi2c1;

/* Handle du timer TIM2 — configuré en Input Capture (non utilisé directement
   dans la logique de mesure ici, les délais reposent sur le DWT) */
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Buffers d'affichage : 16 caractères + terminateur nul pour chaque ligne du LCD */
static char line1[17];  /* Ligne 1 : distance mesurée en cm */
static char line2[17];  /* Ligne 2 : température et humidité */

/* Variables globales de résultat AM2302 (DHT22) */
static int16_t  dht_temp  = 0;  /* Température en dixièmes de °C (ex : 235 = 23.5 °C) */
static uint16_t dht_hum   = 0;  /* Humidité relative en dixièmes de % (ex : 601 = 60.1 %) */
static uint8_t  dht_valid = 0;  /* Indicateur de validité : 1 = lecture OK, 0 = erreur */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* Prototypes des fonctions d'initialisation générées par CubeMX */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ═══════════════════════════════════════════════════════════
 *  Délai µs via DWT (Data Watchpoint and Trace)
 *
 *  Le DWT est un compteur de cycles CPU intégré au Cortex-M4.
 *  Il permet des délais très précis à la microseconde,
 *  indispensables pour les protocoles single-wire des capteurs.
 *  HAL_Delay() ne peut faire que des délais à la milliseconde.
 * ═══════════════════════════════════════════════════════════ */
static void DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;                        /* Capture la valeur courante du compteur de cycles */
    uint32_t ticks = us * (SystemCoreClock / 1000000);   /* Convertit les µs en nombre de cycles CPU */
    while ((DWT->CYCCNT - start) < ticks);               /* Attente active jusqu'à écoulement du délai */
}

/* ═══════════════════════════════════════════════════════════
 *  Pilote LCD — écran 16x2 compatible HD44780 via I2C
 *
 *  Le contrôleur LCD (type AQM1602 ou similaire) reçoit
 *  ses instructions sur le bus I2C1. Chaque trame envoyée
 *  contient un octet de contrôle suivi de la donnée.
 * ═══════════════════════════════════════════════════════════ */

/* Envoie une commande (instruction) au LCD
   L'octet de contrôle 0x80 indique que le prochain octet est une commande */
static void LCD_SendCommand(uint8_t cmd)
{
    uint8_t buf[2] = {0x80, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, buf, 2, 100);
}

/* Envoie un caractère ASCII à afficher à la position courante du curseur
   L'octet de contrôle 0x40 indique que le prochain octet est une donnée (caractère) */
static void LCD_SendChar(uint8_t c)
{
    uint8_t buf[2] = {0x40, c};
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, buf, 2, 100);
}

/* Efface l'écran et remet le curseur en position (0,0)
   La commande 0x01 nécessite un délai d'au moins 1,52 ms d'exécution interne */
static void LCD_Clear(void)
{
    LCD_SendCommand(0x01);
    HAL_Delay(2);   /* Attente fin d'exécution de la commande Clear Display */
}

/* Séquence d'initialisation du LCD selon la datasheet du contrôleur
   Les commandes configurent le mode d'affichage, le contraste et l'alimentation interne */
static void LCD_Init(void)
{
    HAL_Delay(50);              /* Attente stabilisation alimentation LCD au démarrage */
    LCD_SendCommand(0x38);      /* Function Set : 8 bits, 2 lignes, police 5x8 */
    LCD_SendCommand(0x39);      /* Function Set : sélection du jeu d'instructions étendu IS=1 */
    LCD_SendCommand(0x14);      /* Internal OSC frequency : fréquence oscillateur interne */
    LCD_SendCommand(0x70);      /* Contrast Set : bits bas du contraste (0–15) */
    LCD_SendCommand(0x56);      /* Power/Icon/Contrast : activation du booster, bits hauts du contraste */
    LCD_SendCommand(0x6C);      /* Follower Control : activation du suiveur de tension interne */
    HAL_Delay(200);             /* Attente stabilisation du circuit d'alimentation interne */
    LCD_SendCommand(0x38);      /* Retour au jeu d'instructions normal IS=0 */
    LCD_SendCommand(0x0C);      /* Display ON, curseur OFF, pas de clignotement */
    LCD_Clear();                /* Effacement écran pour démarrer proprement */
}

/* Positionne le curseur à la ligne et colonne spécifiées
   row=0 → ligne 1 (adresse DDRAM 0x00), row=1 → ligne 2 (adresse DDRAM 0x40)
   col : position horizontale de 0 à 15 */
static void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? 0x00 : 0x40;       /* Adresse de base selon la ligne */
    LCD_SendCommand(0x80 | (addr + col));            /* Set DDRAM Address : 0x80 | adresse absolue */
}

/* Affiche une chaîne de caractères à partir de la position courante du curseur */
static void LCD_Print(const char *s)
{
    while (*s) LCD_SendChar((uint8_t)*s++);   /* Envoie chaque caractère un par un jusqu'au '\0' */
}

/* ═══════════════════════════════════════════════════════════
 *  Grove Ultrasonic Ranger — protocole single-pin sur PB0
 *
 *  Ce capteur utilise une seule broche pour Trigger ET Echo.
 *  Principe :
 *    1. Le MCU envoie une impulsion HIGH de 10 µs (Trigger).
 *    2. Le capteur émet une rafale ultrasonique (40 kHz).
 *    3. Le capteur maintient la broche HIGH pendant une durée
 *       proportionnelle au temps de vol de l'écho.
 *    4. Distance (cm) = durée_µs / 58
 *       (basé sur la vitesse du son ≈ 343 m/s, aller-retour)
 *
 *  Retourne 9999 en cas de dépassement de portée ou timeout.
 * ═══════════════════════════════════════════════════════════ */

/* Configure PB0 en sortie push-pull pour envoyer le Trigger */
static void ULTRA_SetOutput(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = ULTRA_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;   /* Mode sortie push-pull */
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;  /* Vitesse élevée pour des fronts rapides */
    HAL_GPIO_Init(ULTRA_PORT, &g);
}

/* Configure PB0 en entrée flottante pour lire le signal Echo */
static void ULTRA_SetInput(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = ULTRA_PIN;
    g.Mode = GPIO_MODE_INPUT;   /* Mode entrée */
    g.Pull = GPIO_NOPULL;       /* Pas de résistance interne (signal géré par le capteur) */
    HAL_GPIO_Init(ULTRA_PORT, &g);
}

/* Effectue une mesure de distance complète et retourne la distance en cm */
static uint32_t ULTRA_Read(void)
{
    /* --- Phase 1 : envoi du Trigger --- */
    /* Passage en sortie, envoi d'un pulse HIGH de 10 µs pour déclencher la mesure */
    ULTRA_SetOutput();
    HAL_GPIO_WritePin(ULTRA_PORT, ULTRA_PIN, GPIO_PIN_RESET);   /* S'assure que la ligne est LOW avant */
    DelayUs(2);                                                   /* Petit délai de stabilisation */
    HAL_GPIO_WritePin(ULTRA_PORT, ULTRA_PIN, GPIO_PIN_SET);      /* Front montant → début du Trigger */
    DelayUs(10);                                                  /* Maintien HIGH pendant 10 µs minimum */
    HAL_GPIO_WritePin(ULTRA_PORT, ULTRA_PIN, GPIO_PIN_RESET);    /* Front descendant → fin du Trigger */

    /* --- Phase 2 : attente du signal Echo --- */
    /* Passage en entrée pour écouter la réponse du capteur */
    ULTRA_SetInput();

    /* Attente de la montée du signal Echo (le capteur répond sous ~300 µs après le Trigger)
       Timeout de 5 ms : si dépassé, le capteur ne répond pas → retour 9999 */
    uint32_t start = DWT->CYCCNT;
    uint32_t timeout = 5000 * (SystemCoreClock / 1000000);
    while (HAL_GPIO_ReadPin(ULTRA_PORT, ULTRA_PIN) == GPIO_PIN_RESET)
        if ((DWT->CYCCNT - start) >= timeout) return 9999;

    /* --- Phase 3 : mesure de la durée du pulse Echo --- */
    /* Le signal reste HIGH pendant une durée proportionnelle à la distance
       Timeout de 30 ms → portée maximale théorique ≈ 400 cm */
    uint32_t t_start = DWT->CYCCNT;
    timeout = 30000 * (SystemCoreClock / 1000000);
    while (HAL_GPIO_ReadPin(ULTRA_PORT, ULTRA_PIN) == GPIO_PIN_SET)
        if ((DWT->CYCCNT - t_start) >= timeout) return 9999;

    /* Calcul de la durée du pulse en µs à partir du nombre de cycles écoulés */
    uint32_t elapsed_cycles = DWT->CYCCNT - t_start;
    uint32_t pulse_us = elapsed_cycles / (SystemCoreClock / 1000000);

    /* Conversion en centimètres : durée_µs / 58
       (vitesse du son ~343 m/s → 1 cm ≈ 58 µs aller-retour) */
    return pulse_us / 58;
}

/* ═══════════════════════════════════════════════════════════
 *  AM2302 (DHT22) — protocole single-wire sur PA1
 *
 *  Capteur numérique de température et d'humidité relative.
 *  Protocole propriétaire 1-wire (non standard) :
 *    1. MCU envoie un signal de démarrage LOW > 1 ms.
 *    2. Le capteur répond : LOW 80 µs puis HIGH 80 µs.
 *    3. Le capteur envoie 40 bits de données :
 *         - 16 bits humidité (entier et décimal)
 *         - 16 bits température (entier et décimal, bit15 = signe)
 *         -  8 bits checksum = somme des 4 octets précédents
 *
 *  Encodage des bits :
 *    - "0" : LOW ~50 µs puis HIGH ~26 µs
 *    - "1" : LOW ~50 µs puis HIGH ~70 µs
 *    → Après 40 µs d'attente : si encore HIGH → bit = 1, sinon bit = 0
 *
 *  Les résultats sont stockés dans dht_temp et dht_hum (dixièmes d'unité).
 * ═══════════════════════════════════════════════════════════ */

/* Configure PA1 en sortie push-pull pour envoyer le signal de démarrage */
static void DHT_SetOutput(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = DHT_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;    /* Sortie push-pull */
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;    /* Vitesse faible suffisante pour ce protocole lent */
    HAL_GPIO_Init(DHT_PORT, &g);
}

/* Configure PA1 en entrée avec pull-up pour recevoir les données du capteur
   Le pull-up maintient la ligne à l'état HIGH au repos (ligne idle = HIGH) */
static void DHT_SetInput(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = DHT_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;   /* Pull-up interne requis par le protocole DHT */
    HAL_GPIO_Init(DHT_PORT, &g);
}

/* Attend que la broche PA1 atteigne l'état 'state' dans un délai de timeout_us
   Retourne 1 si l'état est atteint dans le délai, 0 en cas de timeout */
static uint8_t DHT_WaitPin(GPIO_PinState state, uint32_t timeout_us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = timeout_us * (SystemCoreClock / 1000000);
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) != state)
        if ((DWT->CYCCNT - start) >= ticks) return 0;   /* Timeout dépassé → échec */
    return 1;   /* État atteint dans le délai imparti */
}

/* Effectue une lecture complète du capteur AM2302 (DHT22)
   Retourne 1 si la lecture est valide, 0 en cas d'erreur (timeout ou checksum incorrect)
   En cas de succès, met à jour les variables globales dht_temp et dht_hum */
static uint8_t DHT_Read(void)
{
    uint8_t data[5] = {0};   /* Tableau de réception : 5 octets (40 bits) de données brutes */

    /* --- Phase 1 : signal de démarrage envoyé par le MCU ---
       Le MCU tire la ligne LOW pendant 2 ms pour réveiller le capteur
       (minimum requis : > 1 ms selon la datasheet) */
    DHT_SetOutput();
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);   /* Ligne à LOW */
    HAL_Delay(2);                                            /* Maintien LOW pendant 2 ms */
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);      /* Remontée à HIGH */
    DelayUs(30);                                             /* Attente 30 µs avant de relâcher */
    DHT_SetInput();                                          /* Relâche la ligne → le capteur prend le contrôle */

    /* --- Phase 2 : réponse du capteur (handshake) ---
       Le capteur confirme sa présence avec : LOW 80 µs → HIGH 80 µs → LOW (début données) */
    if (!DHT_WaitPin(GPIO_PIN_RESET, 100)) return 0;   /* Attente front descendant (capteur tire LOW) */
    if (!DHT_WaitPin(GPIO_PIN_SET,   100)) return 0;   /* Attente front montant (relâche à HIGH) */
    if (!DHT_WaitPin(GPIO_PIN_RESET, 100)) return 0;   /* Attente du premier front de données */

    /* --- Phase 3 : réception des 40 bits de données ---
       Chaque bit est encodé par la durée du pulse HIGH suivant le LOW de synchronisation */
    for (int i = 0; i < 40; i++)
    {
        /* Attente du front montant : fin du LOW de synchronisation (~50 µs) */
        if (!DHT_WaitPin(GPIO_PIN_SET, 100)) return 0;

        /* Attente de 40 µs pour se placer au milieu de la zone de distinction :
             - Si la broche est encore HIGH après 40 µs → bit = 1 (pulse ≈ 70 µs)
             - Si la broche est déjà LOW  après 40 µs → bit = 0 (pulse ≈ 26 µs) */
        DelayUs(40);
        data[i / 8] <<= 1;   /* Décale les bits déjà lus vers la gauche (MSB en premier) */
        if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET)
            data[i / 8] |= 1;   /* Ajoute un '1' en LSB si la ligne est encore HIGH */

        /* Attente du front descendant (fin du bit) avant de passer au bit suivant */
        if (!DHT_WaitPin(GPIO_PIN_RESET, 100)) return 0;
    }

    /* --- Phase 4 : vérification du checksum ---
       Le 5e octet doit être égal à la somme des 4 premiers tronquée à 8 bits.
       Si le checksum est incorrect, la trame est corrompue → on rejette la mesure */
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
        return 0;   /* Checksum invalide → échec */

    /* --- Décodage des données ---
       Humidité : data[0] = octet de poids fort, data[1] = octet de poids faible
       Température : data[2] = octet de poids fort (bit 15 = signe), data[3] = poids faible
       Les valeurs sont en dixièmes d'unité (ex : 235 → 23.5 °C, 601 → 60.1 %) */
    dht_hum  = ((uint16_t)data[0] << 8) | data[1];
    dht_temp = ((int16_t) data[2] << 8) | data[3];
    return 1;   /* Lecture réussie */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *         Point d'entrée principal — initialise les périphériques puis
  *         boucle en lisant les capteurs et en mettant à jour l'affichage LCD.
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* Initialisation de la HAL (SysTick, groupes de priorité NVIC) */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    /* Configuration de l'horloge système (PLL sur HSI → 80 MHz) */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    /* Initialisation des périphériques générés par CubeMX */
    MX_GPIO_Init();    /* Configuration des broches GPIO (PB0 et PA1) */
    MX_TIM2_Init();    /* Configuration du Timer 2 en mode Input Capture */
    MX_I2C1_Init();    /* Configuration du bus I2C1 pour le LCD */

    /* USER CODE BEGIN 2 */

    /* Activation du compteur de cycles DWT (Data Watchpoint and Trace)
       Nécessaire pour la fonction DelayUs() utilisée par les deux protocoles capteurs.
       DEMCR : registre de contrôle du débogage → active le tracing.
       CYCCNT : compteur 32 bits incrémenté à chaque cycle CPU (80 MHz ici).
       CTRL   : démarre effectivement le compteur. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* Active le module DWT */
    DWT->CYCCNT = 0;                                    /* Remise à zéro du compteur */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;              /* Démarre le compteur de cycles */

    /* Initialisation et message de démarrage sur le LCD */
    LCD_Init();
    LCD_SetCursor(0, 0);
    LCD_Print("  Initialisation");    /* Ligne 1 : message d'accueil */
    LCD_SetCursor(1, 0);
    LCD_Print("    BE_2  OK    ");    /* Ligne 2 : confirmation de démarrage */
    HAL_Delay(1000);                  /* Maintien de l'affichage 1 seconde */
    LCD_Clear();                      /* Effacement avant la boucle principale */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        /* --- Mesure Grove Ultrasonic Ranger ---
           Envoie un Trigger sur PB0 et mesure la durée du pulse Echo retourné.
           La distance est retournée en cm (9999 = hors portée ou pas de réponse). */
        uint32_t dist_cm = ULTRA_Read();
        HAL_Delay(60);   /* Délai minimum entre deux mesures ultrasoniques (~60 ms recommandé) */

        /* --- Mesure AM2302 (DHT22) ---
           Lance un cycle de lecture complet (démarrage + 40 bits + checksum).
           dht_valid = 1 si la lecture est correcte, 0 en cas d'erreur.
           Le capteur nécessite au minimum 2 secondes entre deux lectures. */
        dht_valid = DHT_Read();
        HAL_Delay(2000); /* 1 mesure toutes les 2 s minimum */

        /* --- Construction et affichage ligne 1 : distance en cm ---
           Si le capteur renvoie 9999 (hors portée), on affiche "----".
           Sinon on affiche la valeur numérique formatée sur 4 chiffres. */
        if (dist_cm == 9999)
            snprintf(line1, sizeof(line1), "Dist:  ----  cm ");   /* Hors portée */
        else
            snprintf(line1, sizeof(line1), "Dist: %4lu cm   ", dist_cm);   /* Distance en cm */

        /* --- Construction et affichage ligne 2 : température et humidité ---
           Les valeurs sont stockées en dixièmes : on sépare partie entière et décimale.
           En cas d'échec de lecture DHT, on affiche un message d'erreur. */
        if (dht_valid)
            snprintf(line2, sizeof(line2), "%d.%dC  %d.%d%%   ",
                     dht_temp / 10, (int)(dht_temp % 10),   /* Ex : 235 → "23.5C" */
                     dht_hum  / 10, dht_hum  % 10);          /* Ex : 601 → "60.1%" */
        else
            snprintf(line2, sizeof(line2), "DHT: erreur     ");   /* Lecture échouée */

        /* Mise à jour de l'affichage LCD */
        LCD_SetCursor(0, 0);   /* Curseur en haut à gauche */
        LCD_Print(line1);       /* Affiche la ligne de distance */
        LCD_SetCursor(1, 0);   /* Curseur en bas à gauche */
        LCD_Print(line2);       /* Affiche la ligne température/humidité */

        /* USER CODE END 3 */
    }
}

/**
  * @brief System Clock Configuration
  *        Configure l'horloge système à 80 MHz via PLL sur HSI (16 MHz).
  *        Paramètres PLL : M=1, N=10, R=2 → f_PLL = 16 * 10 / 2 = 80 MHz.
  *        Latence Flash = 4 cycles (requis pour 80 MHz selon la datasheet STM32L4).
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Tension du régulateur interne en Scale 1 (performances maximales, jusqu'à 80 MHz) */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    /* Configuration de l'oscillateur HSI (High Speed Internal, 16 MHz)
       et de la PLL pour atteindre 80 MHz */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;    /* Utilisation du HSI */
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;                 /* Activation du HSI */
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; /* Calibration usine */
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;                 /* Activation PLL */
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;          /* Source PLL = HSI */
    RCC_OscInitStruct.PLL.PLLM            = 1;                          /* Prédiviseur M=1 */
    RCC_OscInitStruct.PLL.PLLN            = 10;                         /* Multiplicateur N=10 */
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;              /* Diviseur P (SAI) */
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;              /* Diviseur Q (USB) */
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;              /* Diviseur R=2 → SYSCLK = 80 MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    /* Configuration des bus d'horloge (AHB, APB1, APB2) tous à 80 MHz */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;   /* SYSCLK = sortie PLL */
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;           /* HCLK = SYSCLK / 1 = 80 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;             /* PCLK1 = HCLK / 1 = 80 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;             /* PCLK2 = HCLK / 1 = 80 MHz */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

/**
  * @brief I2C1 Initialization Function
  *        Configure le bus I2C1 utilisé pour la communication avec le LCD.
  *        Adressage 7 bits, registre Timing calculé pour ~400 kHz (Fast Mode).
  */
static void MX_I2C1_Init(void)
{
    /* USER CODE BEGIN I2C1_Init 0 */
    /* USER CODE END I2C1_Init 0 */
    /* USER CODE BEGIN I2C1_Init 1 */
    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x10909CEC;              /* Valeur Timing pour Fast Mode (~400 kHz) à 80 MHz */
    hi2c1.Init.OwnAddress1      = 0;                       /* Adresse propre du MCU (maître → non utilisée) */
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT; /* Adressage 7 bits (standard) */
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE; /* Pas de double adresse */
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE; /* Pas d'appel général */
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;   /* Clock stretching autorisé (mode maître standard) */
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();
    /* Activation du filtre analogique sur les lignes SDA/SCL pour rejeter les parasites */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        Error_Handler();
    /* Filtre numérique désactivé (0 = aucun filtre supplémentaire) */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
        Error_Handler();
    /* USER CODE BEGIN I2C1_Init 2 */
    /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief TIM2 Initialization Function
  *        Configure TIM2 en mode Input Capture sur le canal 1.
  *        Prescaler = 79 → tick = 1 µs à 80 MHz (80 MHz / (79+1) = 1 MHz).
  *        Note : TIM2 est initialisé mais la mesure du Ranger utilise finalement
  *        le DWT plutôt que le timer IC.
  */
static void MX_TIM2_Init(void)
{
    /* USER CODE BEGIN TIM2_Init 0 */
    /* USER CODE END TIM2_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef      sConfigIC     = {0};

    /* USER CODE BEGIN TIM2_Init 1 */
    /* USER CODE END TIM2_Init 1 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 79;                          /* Prescaler : PCLK / 80 = 1 MHz → 1 tick = 1 µs */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;          /* Comptage croissant */
    htim2.Init.Period            = 65535;                       /* Période maximale (16 bits) */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;      /* Pas de division supplémentaire */
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
        Error_Handler();

    /* Configuration du mode maître (pas de sortie TRGO utilisée ici) */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        Error_Handler();

    /* Configuration du canal 1 en Input Capture sur front montant
       Utilisé pour capturer la durée du pulse Echo (architecture alternative au DWT) */
    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;    /* Capture sur front montant */
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;           /* Entrée directe sur TI1 */
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;                     /* Capture à chaque front */
    sConfigIC.ICFilter    = 0;                                  /* Pas de filtre numérique */
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    /* USER CODE BEGIN TIM2_Init 2 */
    /* USER CODE END TIM2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  *        Configure les deux broches des capteurs avec leurs états initiaux.
  *        PB0 : Grove Ultrasonic Ranger — sortie par défaut, direction commutée en software.
  *        PA1 : AM2302 data line — sortie par défaut, direction commutée en software.
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* Activation des horloges des ports GPIO A et B */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Niveaux logiques initiaux avant la configuration des broches
       PB0 (Ultrasonic) : LOW — ligne au repos avant envoi du Trigger
       PA1 (DHT)        : HIGH — ligne idle du protocole DHT */
    HAL_GPIO_WritePin(ULTRA_PORT, ULTRA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DHT_PORT,   DHT_PIN,   GPIO_PIN_SET);

    /* PB0 — Grove Ultrasonic Ranger
       Initialisé en sortie push-pull ; la direction sera commutée dynamiquement
       par ULTRA_SetOutput() / ULTRA_SetInput() lors de chaque mesure */
    GPIO_InitStruct.Pin   = ULTRA_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ULTRA_PORT, &GPIO_InitStruct);

    /* PA1 — AM2302 data line
       Initialisé en sortie push-pull ; la direction sera commutée dynamiquement
       par DHT_SetOutput() / DHT_SetInput() lors de chaque mesure */
    GPIO_InitStruct.Pin   = DHT_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  Error Handler
  *         Appelé par la HAL en cas d'erreur d'initialisation d'un périphérique.
  *         Désactive les interruptions et bloque le programme dans une boucle infinie
  *         pour permettre l'identification du problème via le débogueur.
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();   /* Désactivation globale des interruptions */
    while (1) {}       /* Blocage → point de breakpoint pour débogage */
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/* Appelé par la HAL si un assert échoue (mode DEBUG uniquement)
   Permet d'identifier le fichier et la ligne à l'origine de l'assertion */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif
