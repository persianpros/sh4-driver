// Tables translate to single byte ASCII
unsigned char UTF_C2[64] =
{
	// table: 0xc2 0xXX (symbols)-> matching symbol
	0x00,  //0x80, nonprintable control, will be skipped
	0x00,  //0x81, nonprintable control, will be skipped
	0x00,  //0x82, nonprintable control, will be skipped
	0x00,  //0x83, nonprintable control, will be skipped
	0x00,  //0x84, nonprintable control, will be skipped
	0x00,  //0x85, nonprintable control, will be skipped
	0x00,  //0x86, nonprintable control, will be skipped
	0x00,  //0x87, nonprintable control, will be skipped
	0x00,  //0x88, nonprintable control, will be skipped
	0x00,  //0x89, nonprintable control, will be skipped
	0x00,  //0x8a, nonprintable control, will be skipped
	0x00,  //0x8b, nonprintable control, will be skipped
	0x00,  //0x8c, nonprintable control, will be skipped
	0x00,  //0x8d, nonprintable control, will be skipped
	0x00,  //0x8e, nonprintable control, will be skipped
	0x00,  //0x8f, nonprintable control, will be skipped

	0x00,  //0x90, nonprintable control, will be skipped
	0x00,  //0x91, nonprintable control, will be skipped
	0x00,  //0x92, nonprintable control, will be skipped
	0x00,  //0x93, nonprintable control, will be skipped
	0x00,  //0x94, nonprintable control, will be skipped
	0x00,  //0x95, nonprintable control, will be skipped
	0x00,  //0x96, nonprintable control, will be skipped
	0x00,  //0x97, nonprintable control, will be skipped
	0x00,  //0x98, nonprintable control, will be skipped
	0x00,  //0x99, nonprintable control, will be skipped
	0x00,  //0x9a, nonprintable control, will be skipped
	0x00,  //0x9b, nonprintable control, will be skipped
	0x00,  //0x9c, nonprintable control, will be skipped
	0x00,  //0x9d, nonprintable control, will be skipped
	0x00,  //0x9e, nonprintable control, will be skipped
	0x00,  //0x9f, nonprintable control, will be skipped

	0x20,  //0xa0, no break space
	0x21,  //0xa1, inverted exclamation mark
	0x63,  //0xa2, cent sign
	0x4c,  //0xa3, pound sign
	0x00,  //0xa4, currency sign
	0x59,  //0xa5, yen sign
	0x2d,  //0xa6, broken bar
	0x00,  //0xa7, paragraph sign
	0x00,  //0xa8, diaeresis
	0x63,  //0xa9, copyright sign
	0x00,  //0xaa, feminine ordinal indicator -
	0x3c,  //0xab, left pointing double angle quotation mark
	0x00,  //0xac, not sign
	0x2d,  //0xad, soft hyphen
	0x72,  //0xae, registered sign
	0x00,  //0xaf, macron

	0x6f,  //0xb0, degree sign
	0x00,  //0xb1, plus-minus sign
	0x32,  //0xb2, superscript two
	0x33,  //0xb3, superscript three
	0x00,  //0xb4, acute accent
	0x75,  //0xb5, micro sign
	0x00,  //0xb6, pilcrow sign
	0x2e,  //0xb7, middle dot
	0x00,  //0xb8, cedilla
	0x31,  //0xb9, superscript one
	0x00,  //0xba, masculine ordinal indicator -
	0x3e,  //0xbb, right pointing double angle quotation mark
	0x00,  //0xbc, vulgar fraction one quarter
	0x00,  //0xbd, vulgar fraction one half
	0x00,  //0xbe, vulgar fraction three quarters
	0x3f   //0xbf, inverted question mark
};

// Accented letters are simply replaced by their unaccented equivalents
// as the display cannot show accents in a nice way
unsigned char UTF_C3[64] =
{
  	// table: 0xc3 0xXX -> non accented ASCII)
	0x41,  //0x80, capital A with accent grave
	0x41,  //0x81, capital A with accent acute
	0x41,  //0x82, capital A with accent circonflexe
	0x41,  //0x83, capital A with accent tilde
	0x41,  //0x84, capital A with accent diaeresis //Ä
	0x41,  //0x85, capital A with accent ring above
	0x41,  //0x86, capital letter AE
	0x43,  //0x87, capital C with cedille
	0x45,  //0x88, capital E with accent grave
	0x45,  //0x89, capital E with accent acute
	0x45,  //0x8a, capital E with accent circonflexe
	0x45,  //0x8b, capital E with accent diaeresis
	0x49,  //0x8c, capital I with accent grave
	0x49,  //0x8d, capital I with accent acute
	0x49,  //0x8e, capital I with accent circonflexe
	0x49,  //0x8f, capital I with accent diaeresis

	0x44,  //0x90, capital D with dash
	0x4e,  //0x91, capital N with accent tilde
	0x4f,  //0x92, capital O with accent grave
	0x4f,  //0x93, capital O with accent acute
	0x4f,  //0x94, capital O with accent circonflexe
	0x4f,  //0x95, capital O with accent tilde
	0x4f,  //0x96, capital O with accent diaeresis //Ö
	0x78,  //0x97, multiplication sign
	0x4f,  //0x98, capital O with stroke
	0x55,  //0x99, capital U with accent grave
	0x55,  //0x9a, capital U with accent acute
	0x55,  //0x9b, capital U with accent circonflexe
	0x55,  //0x9c, capital U with accent diaeresis //Ü
	0x59,  //0x9d, capital Y with accent acute
	0x00,  //0x9e, capital thorn
	0x42,  //0x9f, ringel-S	//ß

	0x61,  //0xa0, small a with accent grave
	0x61,  //0xa1, small a with accent acute
	0x61,  //0xa2, small a with accent circonflexe
	0x61,  //0xa3, small a with accent tilde
	0x61,  //0xa4, small a with accent diaeresis //ä
	0x61,  //0xa5, small a with accent ring above
	0x61,  //0xa6, small letter AE
	0x63,  //0xa7, small c with cedille
	0x65,  //0xa8, small e with accent grave
	0x65,  //0xa9, small e with accent acute
	0x65,  //0xaa, small e with accent circonflexe
	0x65,  //0xab, small e with accent diaeresis
	0x69,  //0xac, small i with accent grave
	0x69,  //0xad, small i with accent acute
	0x69,  //0xae, small i with accent circonflexe
	0x69,  //0xaf, small i with accent diaeresis

	0x45,  //0xb0, small letter ETH
	0x6e,  //0xb1, small n with accent tilde
	0x6f,  //0xb2, small o with accent grave
	0x6f,  //0xb3, small o with accent acute
	0x6f,  //0xb4, small o with accent circonflexe
	0x6f,  //0xb5, small o with accent tilde
	0x6f,  //0xb6, small o with accent diaeresis //ö
	0x2f,  //0xb7, division sign
	0x6f,  //0xb8, small o with stroke
	0x75,  //0xb9, small u with accent grave
	0x75,  //0xba, small u with accent acute
	0x75,  //0xbb, small u with accent circonflexe
	0x75,  //0xbc, small u with accent diaeresis //ü
	0x79,  //0xbd, small y with accent acute
	0x00,  //0xbe, small letter thorn
	0x79,  //0xbf, small y with accent diaresis
};

unsigned char UTF_C4[64] =
{
	// table: 0xc4 -> convert letter to non-accented one    
	0x41,  //0x80, capital A with accent macron
	0x61,  //0x81, small a with accent macron
	0x41,  //0x82, capital A with accent breve
	0x61,  //0x83, small a with accent breve
	0x41,  //0x84, capital A with accent ogonek (polish accented A)
	0x61,  //0x85, small a with accent ogonek (polish accented a)
	0x43,  //0x86, capital C with accent acute (polish accented A)
	0x63,  //0x87, small c with accent acute (polish accented a)
	0x43,  //0x88, capital C with accent circonflexe
	0x63,  //0x89, small c with accent circonflexe
	0x43,  //0x8a, capital C with dot above
	0x63,  //0x8b, small c with dot above
	0x43,  //0x8c, capital C with accent caron
	0x63,  //0x8d, small c with accent caron
	0x44,  //0x8e, capital D with accent caron
	0x64,  //0x8f, small d with accent caron

	0x44,  //0x90, capital D with stroke
	0x64,  //0x91, small d with stroke
	0x45,  //0x92, capital E with accent macron
	0x65,  //0x93, small e with accent macron
	0x45,  //0x94, capital E with accent breve
	0x65,  //0x95, small e with accent breve
	0x45,  //0x96, capital E with dot above
	0x65,  //0x97, small e with dot above
	0x45,  //0x98, capital E with accent ogonek (polish
	0x65,  //0x99, small e with accent ogonek (polish ac
	0x45,  //0x9a, capital E with accent caron
	0x65,  //0x9b, small e with accent caron
	0x47,  //0x9c, capital G with accent circonflexe
	0x67,  //0x9d, small g with accent circonflexe
	0x47,  //0x9e, capital G with accent breve
	0x47,  //0x9f, small g with accent breve

	0x47,  //0xa0, capital G with dot above
	0x67,  //0xa1, small g with dot above
	0x47,  //0xa2, capital G with cedille
	0x67,  //0xa3, small g with cedille
	0x48,  //0xa4, capital H with accent circonflexe
	0x68,  //0xa5, small h with accent circonflexe
	0x48,  //0xa6, capital H with stroke
	0x68,  //0xa7, small h with stroke
	0x49,  //0xa8, capital I with tilde
	0x69,  //0xa9, small i with tilde
	0x49,  //0xaa, capital I with accent macron
	0x69,  //0xab, small i with accent macron
	0x49,  //0xac, capital I with accent breve
	0x69,  //0xad, small i with accent breve
	0x49,  //0xae, capital I with accent ogonek
	0x69,  //0xaf, small i with accent ogonek

	0x49,  //0xb0, capital I with dot above
	0x69,  //0xb1, small i without dot
	0x59,  //0xb2, capital IJ
	0x79,  //0xb3, small ij
	0x4a,  //0xb4, capital J with accent circonflexe
	0x6a,  //0xb5, small j with accent circonflexe
	0x4b,  //0xb6, capital K with cedille
	0x6b,  //0xb7, small k with cedille
	0x6b,  //0xb8, small letter kra
	0x4c,  //0xb9, capital L with accent acute
	0x6c,  //0xba, small l with accent acute
	0x4c,  //0xbb, capital L with cedille
	0x6c,  //0xbc, small l with  cedille
	0x4c,  //0xbd, capital L with accent caron
	0x6c,  //0xbe, small l with accent caron
	0x4c   //0xbf, capital L with middle dot
};

unsigned char UTF_C5[64] =
{
  	// table: 0xc5 -> convert letter to non-accented one
	0x6c,  //0x80, small l with middle dot
	0x4c,  //0x81, capital L with stroke (polish accented L)
	0x6c,  //0x82, small l with stroke (polish accented l)
	0x4e,  //0x83, capital N with accent acute (polish accented N)
	0x6e,  //0x84, small n with accent acute (polish accented n)
	0x4e,  //0x85, capital N with cedille
	0x6e,  //0x86, small n with cedille
	0x4e,  //0x87, capital N with accent caron
	0x6e,  //0x88, small n with accent caron
	0x6e,  //0x89, small n with apostrofe in front
	0x00,  //0x8a, capital letter eng -
	0x00,  //0x8b, small letter eng -
	0x4f,  //0x8c, capital O with macron
	0x6f,  //0x8d, small o with macron
	0x4f,  //0x8e, capital O with breve
	0x6f,  //0x8f, small o with breve

	0x4f,  //0x90, capital O with double accent acute
	0x6f,  //0x91, small o with double accent acute
	0x4f,  //0x92, capital OE
	0x6f,  //0x93, small oe
	0x52,  //0x94, capital R with accent acute
	0x72,  //0x95, small r with accent acute
	0x52,  //0x96, capital R with accent acute
	0x72,  //0x97, small r with accent acute
	0x52,  //0x98, capital R with accent caron
	0x72,  //0x99, small r with accent caron
	0x53,  //0x9a, capital S with accent acute (polish accented S)
	0x73,  //0x9b, small s with accent acute ( polish accented s)
	0x53,  //0x9c, capital S with accent circonflexe
	0x73,  //0x9d, small s with accent circonflexe
	0x53,  //0x9e, capital S with cedille
	0x73,  //0x9f, small s with cedille

	0x53,  //0xa0, capital S with caron
	0x73,  //0xa1, small s with caron
	0x54,  //0xa2, capital T with cedille
	0x74,  //0xa3, small t with cedille
	0x54,  //0xa4, capital T with caron
	0x74,  //0xa5, small t with caron
	0x54,  //0xa6, capital T with stroke
	0x74,  //0xa7, small t with stroke
	0x55,  //0xa8, capital U with tilde
	0x75,  //0xa9, small u with tilde
	0x55,  //0xaa, capital U with macron
	0x75,  //0xab, small u with macron
	0x55,  //0xac, capital U with breve
	0x75,  //0xad, small u with breve
	0x55,  //0xae, capital U with ring above
	0x75,  //0xaf, small u with ring above

	0x55,  //0xb0, capital U with double accent acute
	0x75,  //0xb1, small u with double accent acute
	0x55,  //0xb2, capital U with ogonek
	0x75,  //0xb3, small u with ogonek
	0x57,  //0xb4, capital W with accent circonflexe
	0x77,  //0xb5, small w with accent circonflexe
	0x59,  //0xb6, capital Y with accent circonflexe
	0x79,  //0xb7, small y with accent circonflexe
	0x57,  //0xb8, capital W with accent diaeresis
	0x5a,  //0xb9, capital Z with accent acute (polish accented Zi)
	0x7a,  //0xba, small z with accent acute (polish accented zi)
	0x5a,  //0xbb, capital Z with dot above (polish accented Z)
	0x7a,  //0xbc, small z with dot above (polish accented z)
	0x5a,  //0xbd, capital Z with accent caron
	0x7a,  //0xbe, small z with accent caron
	0x73   //0xbf, small long s
};

unsigned char UTF_D0[64] =
{
  	// Cyrillic I d0 80 - d0 bf (64 characters) -> 0x60 - 0x9f
	0x60,  // Ѐ cyrillic capital letter ie with grave
	0x61,  // Ё cyrillic capital letter io
	0x62,  // Ђ cyrillic capital letter dje
	0x63,  // Ѓ cyrillic capital letter gje
	0x64,  // Є cyrillic capital letter ukrainian ie
	0x65,  // Ѕ cyrillic capital letter dze
	0x66,  // І cyrillic capital letter byelorussian-ukrainian i
	0x67,  // Ї cyrillic capital letter yi
	0x68,  // Ј cyrillic capital letter je
	0x69,  // Љ cyrillic capital letter lje
	0x6a,  // Њ cyrillic capital letter nje
	0x6b,  // Ћ cyrillic capital letter tshe
	0x6c,  // Ќ cyrillic capital letter kje
	0x6d,  // Ѝ cyrillic capital letter i with grave
	0x6e,  // Ў cyrillic capital letter short u
	0x6f,  // Џ cyrillic capital letter dzhe

	0x70,  // А cyrillic capital letter a
	0x71,  // Б cyrillic capital letter be
	0x72,  // В cyrillic capital letter ve
	0x73,  // Г cyrillic capital letter ghe
	0x74,  // Д cyrillic capital letter de
	0x75,  // Е cyrillic capital letter ie
	0x76,  // Ж cyrillic capital letter zhe
	0x77,  // З cyrillic capital letter ze
	0x78,  // И cyrillic capital letter i
	0x79,  // Й cyrillic capital letter short i
	0x7a,  // К cyrillic capital letter ka
	0x7b,  // Л cyrillic capital letter el
	0x7c,  // М cyrillic capital letter em
	0x7d,  // Н cyrillic capital letter en
	0x7e,  // О cyrillic capital letter o
	0x7f,  // П cyrillic capital letter pe

	0x80,  // Р Cyrillic capital letter er
	0x81,  // С Cyrillic capital letter es
	0x82,  // Т Cyrillic capital letter te
	0x83,  // У Cyrillic capital letter u
	0x84,  // Ф Cyrillic capital letter ef
	0x85,  // Х Cyrillic capital letter ha
	0x86,  // Ц Cyrillic capital letter tse
	0x87,  // Ч Cyrillic capital letter che
	0x88,  // Ш Cyrillic capital letter sha
	0x89,  // Щ Cyrillic capital letter shcha
	0x8a,  // Ъ Cyrillic capital letter hard sign
	0x8b,  // Ы Cyrillic capital letter yeru
	0x8c,  // Ь Cyrillic capital letter soft sign
	0x8d,  // Э Cyrillic capital letter e
	0x8e,  // Ю Cyrillic capital letter yu
	0x8f,  // Я Cyrillic capital letter ya

	0x90,  // а Cyrillic small letter a
	0x91,  // б Cyrillic small letter be
	0x92,  // в Cyrillic small letter ve
	0x93,  // г Cyrillic small letter ghe
	0x94,  // д Cyrillic small letter de
	0x95,  // е Cyrillic small letter ie
	0x96,  // ж Cyrillic small letter zhe
	0x97,  // з Cyrillic small letter ze
	0x98,  // и Cyrillic small letter i
	0x99,  // й Cyrillic small letter short i
	0x9a,  // к Cyrillic small letter ka
	0x9b,  // л Cyrillic small letter el
	0x9c,  // м Cyrillic small letter em
	0x9d,  // н Cyrillic small letter en
	0x9e,  // о Cyrillic small letter o
	0x9f   // п Cyrillic small letter pe
};

unsigned char UTF_D1[64] =
  	// Cyrillic ii d1 80 - d0 bf (64 characters) -> 0xa0 - 0xdf
{
	0xa0,  // р Cyrillic small letter er
	0xa1,  // с Cyrillic small letter es
	0xa2,  // т Cyrillic small letter te
	0xa3,  // у Cyrillic small letter u
	0xa4,  // ф Cyrillic small letter ef
	0xa5,  // х Cyrillic small letter ha
	0xa6,  // ц Cyrillic small letter tse
	0xa7,  // ч Cyrillic small letter che
	0xa8,  // ш Cyrillic small letter sha
	0xa9,  // щ Cyrillic small letter shcha
	0xaa,  // ъ Cyrillic small letter hard sign
	0xab,  // ы Cyrillic small letter yeru
	0xac,  // ь Cyrillic small letter soft sign
	0xad,  // э Cyrillic small letter e
	0xae,  // ю Cyrillic small letter yu
	0xaf,  // я Cyrillic small letter ya

	0xb0,  // ѐ Cyrillic small letter ie with grave
	0xb1,  // ё Cyrillic small letter io
	0xb2,  // ђ Cyrillic small letter dje
	0xb3,  // ѓ Cyrillic small letter gje
	0xb4,  // є Cyrillic small letter ukrainian ie
	0xb5,  // ѕ Cyrillic small letter dze
	0xb6,  // і Cyrillic small letter byelorussian-ukrainian i
	0xb7,  // ї Cyrillic small letter yi
	0xb8,  // ј Cyrillic small letter je
	0xb9,  // љ Cyrillic small letter lje
	0xba,  // њ Cyrillic small letter nje
	0xbb,  // ћ Cyrillic small letter tshe
	0xbc,  // ќ Cyrillic small letter kje
	0xbd,  // ѝ Cyrillic small letter i with grave
	0xbe,  // ў Cyrillic small letter short u
	0xbf,  // џ Cyrillic small letter dzhe

	0xc0,  // Ѡ Cyrillic capital letter omega
	0xc1,  // ѡ Cyrillic small letter omega
	0xc2,  // Ѣ Cyrillic capital letter yat
	0xc3,  // ѣ Cyrillic small letter yat
	0xc4,  // Ѥ Cyrillic capital letter iotified e
	0xc5,  // ѥ Cyrillic small letter iotified e
	0xc6,  // Ѧ Cyrillic capital letter little yus
	0xc7,  // ѧ Cyrillic small letter little yus
	0xc8,  // Ѩ Cyrillic capital letter iotified little yus
	0xc9,  // ѩ Cyrillic small letter iotified little yus
	0xca,  // Ѫ Cyrillic capital letter big yus
	0xcb,  // ѫ Cyrillic small letter big yus
	0xcc,  // Ѭ Cyrillic capital letter iotified big yus
	0xcd,  // ѭ Cyrillic small letter iotified big yus
	0xce,  // Ѯ Cyrillic capital letter ksi
	0xcf,  // ѯ Cyrillic small letter ksi

	0xd0,  // Ѱ Cyrillic capital letter psi
	0xd1,  // ѱ Cyrillic small letter psi
	0xd2,  // Ѳ Cyrillic capital letter fita
	0xd3,  // ѳ Cyrillic small letter fita
	0xd4,  // Ѵ Cyrillic capital letter izhitsa
	0xd5,  // ѵ Cyrillic small letter izhitsa
	0xd6,  // Ѷ Cyrillic capital letter izhitsa with double grave accent
	0xd7,  // ѷ Cyrillic small letter izhitsa with double grave accent
	0xd8,  // Ѹ Cyrillic capital letter uk
	0xd9,  // ѹ Cyrillic small letter uk
	0xda,  // Ѻ Cyrillic capital letter round omega
	0xdb,  // ѻ Cyrillic small letter round omega
	0xdc,  // Ѽ Cyrillic capital letter omega with titlo
	0xdd,  // ѽ Cyrillic small letter omega with titlo
	0xde,  // Ѿ Cyrillic capital letter ot
	0xdf   // ѿ Cyrillic small letter ot

};
// vim:ts=4
