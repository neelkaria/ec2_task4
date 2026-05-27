// ============================================================================
// SimpleJson.h
// Einfacher JSON Parser und Serializer fuer flache Objekte (keine Verschachtelung)
// Unterstuetzte Typen: String, int, float, bool
//
// Beispiel:  {"potValue":512,"blinkEnabled":true,"name":"Pico"}
//
// Institut fuer Elektromobilitaet (IEM), Hochschule Ravensburg-Weingarten
// ============================================================================
#pragma once
#include <Arduino.h>

class SimpleJson {
public:
    // Maximale Anzahl Key-Value Paare und Laengen
    static const int MAX_PAIRS   = 16;
    static const int MAX_KEY_LEN = 32;
    static const int MAX_VAL_LEN = 64;

    SimpleJson() : _count(0) {
        clear();
    }

    // ── Speicher zuruecksetzen ──────────────────────────────────────
    void clear() {
        _count = 0;
        for (int i = 0; i < MAX_PAIRS; i++) {
            _pairs[i].key[0]    = '\0';
            _pairs[i].value[0]  = '\0';
            _pairs[i].isString  = false;
            _pairs[i].occupied  = false;
        }
    }

    // ================================================================
    //  PARSER:  JSON String  ->  Key/Value Paare
    // ================================================================
    bool parse(const char* json) {
        clear();
        int pos = 0;

        skipWS(json, pos);
        if (json[pos] != '{') return false;   // muss mit { starten
        pos++;

        while (json[pos] != '\0') {
            skipWS(json, pos);

            // Ende des Objekts erreicht
            if (json[pos] == '}') return true;

            // Komma zwischen Paaren ueberspringen
            if (json[pos] == ',') {
                pos++;
                skipWS(json, pos);
            }

            // Kapazitaet pruefen
            if (_count >= MAX_PAIRS) return false;
            Pair& p = _pairs[_count];

            // ── Key lesen (immer ein String in Anfuehrungszeichen) ──
            if (!parseQuoted(json, pos, p.key, MAX_KEY_LEN)) return false;

            // ── Doppelpunkt erwarten ────────────────────────────────
            skipWS(json, pos);
            if (json[pos] != ':') return false;
            pos++;
            skipWS(json, pos);

            // ── Value lesen ─────────────────────────────────────────
            if (json[pos] == '"') {
                // String Wert
                if (!parseQuoted(json, pos, p.value, MAX_VAL_LEN)) return false;
                p.isString = true;
            } else {
                // Zahl, bool oder null
                if (!parseRaw(json, pos, p.value, MAX_VAL_LEN)) return false;
                p.isString = false;
            }

            p.occupied = true;
            _count++;
        }

        return false;  // kein schliessendes } gefunden
    }

    // ================================================================
    //  GETTER:  Werte nach Key abfragen
    // ================================================================

    bool hasKey(const char* key) const {
        return findIndex(key) >= 0;
    }

    const char* getString(const char* key, const char* def = "") const {
        int i = findIndex(key);
        return (i >= 0) ? _pairs[i].value : def;
    }

    int getInt(const char* key, int def = 0) const {
        int i = findIndex(key);
        return (i >= 0) ? atoi(_pairs[i].value) : def;
    }

    float getFloat(const char* key, float def = 0.0f) const {
        int i = findIndex(key);
        return (i >= 0) ? (float)atof(_pairs[i].value) : def;
    }

    bool getBool(const char* key, bool def = false) const {
        int i = findIndex(key);
        if (i < 0) return def;
        // "true" oder "1" gelten als true
        return (strcmp(_pairs[i].value, "true") == 0
             || strcmp(_pairs[i].value, "1") == 0);
    }

    // ================================================================
    //  SETTER:  Werte setzen (zum Aufbauen eines JSON Objekts)
    // ================================================================

    void setString(const char* key, const char* value) {
        int i = getOrCreate(key);
        if (i < 0) return;
        strncpy(_pairs[i].value, value, MAX_VAL_LEN - 1);
        _pairs[i].value[MAX_VAL_LEN - 1] = '\0';
        _pairs[i].isString = true;
    }

    void setInt(const char* key, int value) {
        int i = getOrCreate(key);
        if (i < 0) return;
        snprintf(_pairs[i].value, MAX_VAL_LEN, "%d", value);
        _pairs[i].isString = false;
    }

    void setFloat(const char* key, float value, int decimals = 2) {
        int i = getOrCreate(key);
        if (i < 0) return;
        // snprintf statt dtostrf (SAMD-kompatibel)
        char fmt[8];
        snprintf(fmt, sizeof(fmt), "%%.%df", decimals);
        snprintf(_pairs[i].value, MAX_VAL_LEN, fmt, (double)value);
        _pairs[i].isString = false;
    }

    void setBool(const char* key, bool value) {
        int i = getOrCreate(key);
        if (i < 0) return;
        strcpy(_pairs[i].value, value ? "true" : "false");
        _pairs[i].isString = false;
    }

    // ================================================================
    //  SERIALIZER:  Key/Value Paare  ->  JSON String
    // ================================================================

    // Variante 1: In char-Array schreiben (kein Heap, embedded-freundlich)
    // Rueckgabe: Anzahl geschriebener Zeichen (ohne Nullterminator)
    int toCharArray(char* buf, int bufSize) const {
        int pos = 0;

        auto put = [&](char c) {
            if (pos < bufSize - 1) buf[pos++] = c;
        };
        auto putStr = [&](const char* s) {
            while (*s && pos < bufSize - 1) buf[pos++] = *s++;
        };

        put('{');

        bool first = true;
        for (int i = 0; i < _count; i++) {
            if (!_pairs[i].occupied) continue;

            if (!first) put(',');
            first = false;

            // Key
            put('"');
            putStr(_pairs[i].key);
            put('"');
            put(':');

            // Value
            if (_pairs[i].isString) put('"');
            putStr(_pairs[i].value);
            if (_pairs[i].isString) put('"');
        }

        put('}');
        buf[pos] = '\0';
        return pos;
    }

    // Variante 2: Als Arduino String (komfortabler, aber nutzt Heap)
    String toString() const {
        char buf[512];
        toCharArray(buf, sizeof(buf));
        return String(buf);
    }

    // Anzahl gespeicherter Paare
    int count() const { return _count; }

    // ================================================================
    //  DEBUG:  Alle Paare auf Serial ausgeben
    // ================================================================
    void dump(Stream& out) const {
        out.println("SimpleJson Inhalt:");
        for (int i = 0; i < _count; i++) {
            if (!_pairs[i].occupied) continue;
            out.print("  ");
            out.print(_pairs[i].key);
            out.print(" = ");
            if (_pairs[i].isString) out.print('"');
            out.print(_pairs[i].value);
            if (_pairs[i].isString) out.print('"');
            out.println();
        }
    }

private:
    // ── Interne Datenstruktur ───────────────────────────────────────
    struct Pair {
        char key[MAX_KEY_LEN];
        char value[MAX_VAL_LEN];
        bool isString;      // true = Wert war in Anfuehrungszeichen
        bool occupied;
    };

    Pair _pairs[MAX_PAIRS];
    int  _count;

    // ── Hilfsfunktionen ─────────────────────────────────────────────

    // Index eines Keys finden, oder -1
    int findIndex(const char* key) const {
        for (int i = 0; i < _count; i++) {
            if (_pairs[i].occupied && strcmp(_pairs[i].key, key) == 0) {
                return i;
            }
        }
        return -1;
    }

    // Index eines Keys finden oder neuen Slot anlegen
    int getOrCreate(const char* key) {
        int i = findIndex(key);
        if (i >= 0) return i;
        if (_count >= MAX_PAIRS) return -1;
        i = _count++;
        strncpy(_pairs[i].key, key, MAX_KEY_LEN - 1);
        _pairs[i].key[MAX_KEY_LEN - 1] = '\0';
        _pairs[i].occupied = true;
        return i;
    }

    // Whitespace ueberspringen
    void skipWS(const char* s, int& pos) {
        while (s[pos] == ' ' || s[pos] == '\t'
            || s[pos] == '\n' || s[pos] == '\r') {
            pos++;
        }
    }

    // String in Anfuehrungszeichen parsen: "abc" -> abc
    bool parseQuoted(const char* s, int& pos, char* out, int maxLen) {
        if (s[pos] != '"') return false;
        pos++;  // oeffnendes Anfuehrungszeichen

        int i = 0;
        while (s[pos] != '\0' && s[pos] != '"' && i < maxLen - 1) {
            if (s[pos] == '\\' && s[pos + 1] != '\0') {
                pos++;  // Escape-Sequenz
                switch (s[pos]) {
                    case 'n':  out[i++] = '\n'; break;
                    case 't':  out[i++] = '\t'; break;
                    case '"':  out[i++] = '"';  break;
                    case '\\': out[i++] = '\\'; break;
                    default:   out[i++] = s[pos]; break;
                }
            } else {
                out[i++] = s[pos];
            }
            pos++;
        }
        out[i] = '\0';

        if (s[pos] == '"') { pos++; return true; }
        return false;  // kein schliessendes Anfuehrungszeichen
    }

    // Rohwert parsen (Zahl, bool, null) bis zum naechsten , oder }
    bool parseRaw(const char* s, int& pos, char* out, int maxLen) {
        int i = 0;
        while (s[pos] != '\0' && s[pos] != ',' && s[pos] != '}'
            && s[pos] != ' '  && s[pos] != '\t'
            && i < maxLen - 1) {
            out[i++] = s[pos++];
        }
        out[i] = '\0';
        return (i > 0);
    }
};
