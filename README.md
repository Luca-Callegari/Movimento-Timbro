# Sintesi e Analisi Cinematica di una Macchina Stampatrice 📐⚙️

Questo repository contiene lo sviluppo e la codifica in ambiente MATLAB di un progetto di **Meccanica Applicata alle Macchine**. L'obiettivo è progettare e validare cinematicamente un **quadrilatero articolato** destinato a muovere il timbro di una macchina stampatrice tra due posizioni operative distinte su piani ortogonali.

---

## 1. Il Problema Ingegneristico

Il sistema deve movimentare un timbro facendolo passare da una **Posizione 1** (dove tocca il tampone dell'inchiostro) a una **Posizione 2** (dove effettua la stampa sul foglio). I due piani di lavoro sono ortogonali tra loro e disposti secondo specifiche geometrie di progetto.

L'intero movimento deve essere guidato da un quadrilatero articolato, garantendo che il timbro assuma l'orientamento corretto e non urti i componenti della macchina durante la transizione.

---

## 2. Come Funziona il Codice (La Logica Matematica)

Il codice MATLAB (`codice.m`) implementa la sintesi geometrica e la successiva analisi del meccanismo. Ecco le fasi logiche principali tradotte in formule e passaggi matematici:

### 📍 Definizione delle Configurazioni di Lavoro
Il programma inizia definendo le coordinate cartesiane dei punti d'interesse per le due configurazioni limite richieste (iniziale e finale), identificando i segmenti di movimento per i punti caratteristici del meccanismo ($A_1, A_2$ e $B_1, B_2$).

### 📐 Calcolo delle Mediane Ortogonali
Per trovare la posizione ottimale delle cerniere fisse a terra ($A_0$ e $B_0$), il codice sfrutta le proprietà geometriche dei luoghi dei punti. Calcola i punti medi ($M_A, M_B$) e le pendenze ortogonali ($m_{\perp A}, m_{\perp B}$) dei segmenti di transizione:

$$m_{\perp} = -\frac{1}{m}$$

Attraverso le intercette delle rette ortogonali ($q_{\perp A}, q_{\perp B}$), il codice imposta le funzioni matematiche per definire la retta dei centri:

$$y_{A0}(x_{A0}) = m_{\perp A} \cdot x_{A0} + q_{\perp A}$$

### ⭕ Sintesi dei Cerchi per i Tre Punti
All'interno dello script è implementata una funzione ad hoc (`cerchioTrePunti`) che risolve un sistema lineare per determinare il centro ($x_c, y_c$) e il raggio ($R$) della traiettoria circolare passante per tre punti specifici nello spazio. Il sistema lineare risolto dal codice assume la forma:

$$\begin{pmatrix} 2(x_2 - x_1) & 2(y_2 - y_1) \\ 2(x_3 - x_1) & 2(y_3 - y_1) \end{pmatrix} \begin{pmatrix} x_c \\ y_c \end{pmatrix} = \begin{pmatrix} x_2^2 - x_1^2 + y_2^2 - y_1^2 \\ x_3^2 - x_1^2 + y_3^2 - y_1^2 \end{pmatrix}$$

Una volta ricavato il centro tramite inversione di matrice, il raggio viene calcolato semplicemente come distanza euclidea da uno dei punti:

$$R = \sqrt{(x_c - x_1)^2 + (y_c - y_1)^2}$$

### 📊 Plot e Validazione Grafica
L'ultima sezione del codice si occupa di plottare i risultati, tracciare i punti di flesso del meccanismo e scalare correttamente gli assi visivi (`pbaspect` e limiti `xlim`/`ylim`) per validare visivamente che il quadrilatero non soffra di problemi di bloccaggio cinematico o singolarità durante l'arco di movimento.

---

## 3. Struttura del Repository

* `codice.m`: Lo script principale in MATLAB che esegue i calcoli geometrici, la sintesi delle cerniere e genera i grafici di validazione del meccanismo.
* `README.md`: Questa documentazione.

---
**Corso:** Meccanica Applicata alle Macchine  
**Anno Accademico:** 2025/2026  
**Università:** Università degli Studi di Roma "Tor Vergata"
