#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct CommandContract {
    argparse_args: Vec<String>,
    hydra_overrides: Vec<String>,
}

impl CommandContract {
    #[must_use]
    pub fn new(argparse_args: Vec<String>, hydra_overrides: Vec<String>) -> Self {
        Self {
            argparse_args,
            hydra_overrides,
        }
    }

    #[must_use]
    pub fn argparse_args(&self) -> &[String] {
        &self.argparse_args
    }

    #[must_use]
    pub fn hydra_overrides(&self) -> &[String] {
        &self.hydra_overrides
    }

    #[must_use]
    pub fn argv(&self) -> Vec<String> {
        self.argparse_args()
            .iter()
            .chain(self.hydra_overrides().iter())
            .cloned()
            .collect()
    }

    #[must_use]
    pub fn shell_command(&self, executable: &str) -> String {
        std::iter::once(executable.to_string())
            .chain(self.argv())
            .map(|arg| quote_shell_token(&arg))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

pub const REDACTED: &str = "<redacted>";

/// Credential vocabulary: a key is secret when any component is one of these.
///
/// Identical to `srb.utils.redaction.SECRET_KEY_COMPONENTS`. Keys are split
/// into segments at every character outside `[A-Za-z0-9_-]`, and segments
/// into lowercased components at `_`, `-` and lower-to-upper camelCase
/// boundaries. Keep in sync with `srb.utils.redaction`; nothing checks that
/// the two agree.
const SECRET_COMPONENTS: &[&str] = &[
    "accesskey",
    "apikey",
    "auth",
    "authorization",
    "bearer",
    "cookie",
    "cookies",
    "credential",
    "credentials",
    "dsn",
    "passphrase",
    "passwd",
    "password",
    "passwords",
    "pat",
    "privatekey",
    "secret",
    "secrets",
    "token",
];

const SECRET_KEY_QUALIFIERS: &[&str] = &[
    "access",
    "api",
    "auth",
    "client",
    "encryption",
    "private",
    "secret",
    "session",
    "signing",
    "ssh",
];

fn is_segment_byte(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || byte == b'_' || byte == b'-'
}

fn segment_components(segment: &str) -> Vec<String> {
    let mut components = Vec::new();
    let mut current = String::new();
    let mut previous: Option<char> = None;
    let characters: Vec<char> = segment.chars().collect();
    for (index, &character) in characters.iter().enumerate() {
        if character == '_' || character == '-' {
            if !current.is_empty() {
                components.push(std::mem::take(&mut current));
            }
        } else {
            let lower_or_digit_boundary = character.is_ascii_uppercase()
                && previous.is_some_and(|p| p.is_ascii_lowercase() || p.is_ascii_digit());
            let acronym_boundary = character.is_ascii_uppercase()
                && previous.is_some_and(|p| p.is_ascii_uppercase())
                && characters
                    .get(index + 1)
                    .is_some_and(|next| next.is_ascii_lowercase());
            if (lower_or_digit_boundary || acronym_boundary) && !current.is_empty() {
                components.push(std::mem::take(&mut current));
            }
            current.push(character.to_ascii_lowercase());
        }
        previous = Some(character);
    }
    if !current.is_empty() {
        components.push(current);
    }
    components
}

fn key_segments(key: &str) -> impl Iterator<Item = &str> {
    key.split(|character: char| !(character.is_ascii() && is_segment_byte(character as u8)))
}

/// True when an argument key names a credential-like value.
///
/// Mirrors `srb.utils.redaction.is_secret_key`: `HF_TOKEN`, `authToken`,
/// `wandb.api_key`, `wandb.key` and `++wandb.key` are secrets, while `author`,
/// `monkey`, `keyboard`, `obs_key`, `sort_key` and `max_tokens` are not.
#[must_use]
pub fn is_secret_key(key: &str) -> bool {
    key_segments(key).any(|segment| {
        let components = segment_components(segment);
        components
            .iter()
            .any(|component| SECRET_COMPONENTS.contains(&component.as_str()))
            || components == ["key"]
            || (components.len() >= 2
                && components[components.len() - 1] == "key"
                && SECRET_KEY_QUALIFIERS.contains(&components[components.len() - 2].as_str()))
    })
}

fn is_flag_token(token: &str) -> bool {
    token.starts_with('-') && token.len() > 1
}

fn is_negated_flag(token: &str) -> bool {
    token.starts_with("--")
        && key_segments(token)
            .next()
            .map(segment_components)
            .is_some_and(|components| components.first().is_some_and(|first| first == "no"))
}

/// True for a token that is exactly an option spelling: `--`, a single-letter
/// short option (`-v`), or a bare lowercase long option (`--headless`,
/// `--wandb.mode`). Mirrors `srb.utils.redaction._is_exact_option`.
fn is_exact_option_token(token: &str) -> bool {
    if token == "--" {
        return true;
    }
    if let Some(name) = token.strip_prefix("--") {
        return name.as_bytes().first().is_some_and(u8::is_ascii_lowercase)
            && name.split(['-', '_', '.']).all(|segment| {
                !segment.is_empty()
                    && segment
                        .bytes()
                        .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit())
            });
    }
    let bytes = token.as_bytes();
    bytes.len() == 2 && bytes[0] == b'-' && bytes[1].is_ascii_alphabetic()
}

fn is_key_shaped(key: &str) -> bool {
    !key.is_empty()
        && key.bytes().all(|byte| {
            byte.is_ascii_alphanumeric()
                || matches!(
                    byte,
                    b'_' | b'.' | b'-' | b'/' | b'+' | b'~' | b'@' | b'[' | b']'
                )
        })
}

fn redact_url_userinfo(value: &str) -> String {
    let bytes = value.as_bytes();
    let mut result = String::with_capacity(value.len());
    let mut kept_from = 0;
    let mut search = 0;
    while let Some(offset) = value[search..].find("://") {
        let marker = search + offset;
        let start = marker + 3;
        search = start;
        if marker == 0 || !bytes[marker - 1].is_ascii_alphanumeric() {
            continue;
        }
        let mut end = start;
        while end < bytes.len()
            && !matches!(bytes[end], b'/' | b'?' | b'#')
            && !bytes[end].is_ascii_whitespace()
        {
            end += 1;
        }
        if let Some(at_offset) = value[start..end].rfind('@') {
            if at_offset > 0 {
                result.push_str(&value[kept_from..start]);
                result.push_str(REDACTED);
                kept_from = start + at_offset;
            }
        }
        search = end.max(start);
    }
    result.push_str(&value[kept_from..]);
    result
}

fn redact_query_parameters(value: &str) -> String {
    let bytes = value.as_bytes();
    let mut result = String::with_capacity(value.len());
    let mut kept_from = 0;
    let mut search = 0;
    while let Some(offset) = value[search..].find('?') {
        let marker = search + offset;
        let mut end = marker + 1;
        while end < bytes.len() && bytes[end] != b'#' && !bytes[end].is_ascii_whitespace() {
            end += 1;
        }
        let mut changed = false;
        let parameters: Vec<String> = value[marker + 1..end]
            .split('&')
            .map(|parameter| match parameter.split_once('=') {
                Some((key, _)) if is_secret_key(&decode_query_key(key)) => {
                    changed = true;
                    format!("{key}={REDACTED}")
                }
                _ => parameter.to_string(),
            })
            .collect();
        if changed {
            result.push_str(&value[kept_from..marker]);
            result.push('?');
            result.push_str(&parameters.join("&"));
            kept_from = end;
        }
        search = end;
    }
    result.push_str(&value[kept_from..]);
    result
}

fn decode_query_key(key: &str) -> String {
    let bytes = key.as_bytes();
    let mut decoded = Vec::with_capacity(bytes.len());
    let mut index = 0;
    while index < bytes.len() {
        match bytes[index] {
            b'+' => decoded.push(b' '),
            b'%' => {
                let escaped = bytes.get(index + 1..index + 3).and_then(|hex| {
                    let high = char::from(hex[0]).to_digit(16)?;
                    let low = char::from(hex[1]).to_digit(16)?;
                    u8::try_from(high * 16 + low).ok()
                });
                if let Some(byte) = escaped {
                    decoded.push(byte);
                    index += 3;
                    continue;
                }
                decoded.push(b'%');
            }
            byte => decoded.push(byte),
        }
        index += 1;
    }
    String::from_utf8_lossy(&decoded).into_owned()
}

fn redact_bearer(value: &str) -> String {
    let bytes = value.as_bytes();
    let mut result = String::with_capacity(value.len());
    let mut kept_from = 0;
    let mut index = 0;
    while index + 6 <= bytes.len() {
        if bytes[index..index + 6].eq_ignore_ascii_case(b"bearer")
            && (index == 0 || !bytes[index - 1].is_ascii_alphanumeric())
        {
            let mut gap = index + 6;
            while gap < bytes.len() && bytes[gap].is_ascii_whitespace() {
                gap += 1;
            }
            let mut end = gap;
            while end < bytes.len() && !bytes[end].is_ascii_whitespace() {
                end += 1;
            }
            if gap > index + 6 && end > gap {
                result.push_str(&value[kept_from..gap]);
                result.push_str(REDACTED);
                kept_from = end;
                index = end;
                continue;
            }
        }
        index += 1;
    }
    result.push_str(&value[kept_from..]);
    result
}

/// Mask known raw credential formats even when the value uses an innocuous key.
/// Keep these minimum lengths aligned with `_RAW_CREDENTIAL` in Python.
fn redact_raw_credentials(value: &str) -> String {
    let bytes = value.as_bytes();
    let mut result = String::with_capacity(value.len());
    let mut kept_from = 0;
    let mut index = 0;
    while index < bytes.len() {
        if index > 0 && bytes[index - 1].is_ascii_alphanumeric() {
            index += 1;
            continue;
        }

        let candidate = &bytes[index..];
        let (prefix_len, minimum_body, alphabet): (usize, usize, fn(u8) -> bool) =
            if candidate.starts_with(b"hf_") {
                (3, 20, |byte| byte.is_ascii_alphanumeric())
            } else if candidate.starts_with(b"ghp_") {
                (4, 20, |byte| byte.is_ascii_alphanumeric())
            } else if candidate.starts_with(b"sk-") {
                (3, 20, |byte| {
                    byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-')
                })
            } else if candidate.starts_with(b"glpat-") {
                (6, 20, |byte| {
                    byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-')
                })
            } else if candidate.starts_with(b"AKIA") {
                let end = index + 20;
                if candidate.len() >= 20
                    && candidate[4..20]
                        .iter()
                        .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit())
                    && (end == bytes.len() || !bytes[end].is_ascii_alphanumeric())
                {
                    result.push_str(&value[kept_from..index]);
                    result.push_str(REDACTED);
                    kept_from = end;
                    index = end;
                    continue;
                }
                index += 1;
                continue;
            } else {
                index += 1;
                continue;
            };

        let mut end = index + prefix_len;
        while end < bytes.len() && alphabet(bytes[end]) {
            end += 1;
        }
        if end - (index + prefix_len) >= minimum_body
            && (end == bytes.len() || !bytes[end].is_ascii_alphanumeric())
        {
            result.push_str(&value[kept_from..index]);
            result.push_str(REDACTED);
            kept_from = end;
            index = end;
        } else {
            index += 1;
        }
    }
    result.push_str(&value[kept_from..]);
    result
}

fn split_mapping_entries(value: &str) -> Vec<&str> {
    let bytes = value.as_bytes();
    let mut entries = Vec::new();
    let mut start = 0;
    let mut depth: usize = 0;
    let mut quote = None;
    let mut escaped = false;
    for (index, &byte) in bytes.iter().enumerate() {
        if let Some(active_quote) = quote {
            if escaped {
                escaped = false;
            } else if byte == b'\\' {
                escaped = true;
            } else if byte == active_quote {
                quote = None;
            }
        } else if byte == b'\'' || byte == b'"' {
            quote = Some(byte);
        } else if byte == b'{' {
            depth += 1;
        } else if byte == b'}' {
            depth = depth.saturating_sub(1);
        } else if byte == b',' && depth == 0 {
            entries.push(&value[start..index]);
            start = index + 1;
        }
    }
    entries.push(&value[start..]);
    entries
}

fn mapping_separator(entry: &str) -> Option<usize> {
    let mut depth: usize = 0;
    let mut quote = None;
    let mut escaped = false;
    for (index, &byte) in entry.as_bytes().iter().enumerate() {
        if let Some(active_quote) = quote {
            if escaped {
                escaped = false;
            } else if byte == b'\\' {
                escaped = true;
            } else if byte == active_quote {
                quote = None;
            }
        } else if byte == b'\'' || byte == b'"' {
            quote = Some(byte);
        } else if byte == b'{' {
            depth += 1;
        } else if byte == b'}' {
            depth = depth.saturating_sub(1);
        } else if depth == 0 && (byte == b':' || byte == b'=') {
            return Some(index);
        }
    }
    None
}

fn redact_nested_credentials(value: &str) -> String {
    let Some(opening) = value.find('{') else {
        return value.to_string();
    };
    let bytes = value.as_bytes();
    let mut depth: usize = 0;
    let mut quote = None;
    let mut escaped = false;
    let mut closing = None;
    for (index, &byte) in bytes.iter().enumerate().skip(opening) {
        if let Some(active_quote) = quote {
            if escaped {
                escaped = false;
            } else if byte == b'\\' {
                escaped = true;
            } else if byte == active_quote {
                quote = None;
            }
        } else if byte == b'\'' || byte == b'"' {
            quote = Some(byte);
        } else if byte == b'{' {
            depth += 1;
        } else if byte == b'}' {
            depth -= 1;
            if depth == 0 {
                closing = Some(index);
                break;
            }
        }
    }
    let Some(closing) = closing else {
        return format!(
            "{}{{{}",
            &value[..opening],
            redact_nested_credentials(&value[opening + 1..])
        );
    };

    let mut mapped = String::from("{");
    for (index, entry) in split_mapping_entries(&value[opening + 1..closing])
        .into_iter()
        .enumerate()
    {
        if index > 0 {
            mapped.push(',');
        }
        let Some(separator) = mapping_separator(entry) else {
            mapped.push_str(&redact_nested_credentials(entry));
            continue;
        };
        let raw_key = entry[..separator].trim();
        let key = raw_key
            .strip_prefix('\'')
            .and_then(|key| key.strip_suffix('\''))
            .or_else(|| {
                raw_key
                    .strip_prefix('"')
                    .and_then(|key| key.strip_suffix('"'))
            })
            .unwrap_or(raw_key);
        let raw_value = &entry[separator + 1..];
        mapped.push_str(&entry[..=separator]);
        if is_secret_key(key) {
            let leading = raw_value.len() - raw_value.trim_start().len();
            let trailing = raw_value.len() - raw_value.trim_end().len();
            mapped.push_str(&raw_value[..leading]);
            mapped.push_str(REDACTED);
            if trailing > 0 {
                mapped.push_str(&raw_value[raw_value.len() - trailing..]);
            }
        } else {
            mapped.push_str(&redact_nested_credentials(raw_value));
        }
    }
    mapped.push('}');
    mapped.push_str(&redact_nested_credentials(&value[closing + 1..]));
    format!("{}{}", &value[..opening], mapped)
}

#[must_use]
pub fn redact_secret_text(value: &str) -> String {
    let value = redact_nested_credentials(value);
    let value = redact_url_userinfo(&value);
    let value = redact_query_parameters(&value);
    if let Some(colon) = value.find(':') {
        if colon > 0 {
            let whitespace: &[char] = &[' ', '\t', '\n', '\x0c', '\r'];
            let name = value[..colon].trim_matches(whitespace);
            let rest = value[colon + 1..].trim_matches(whitespace);
            if !rest.is_empty()
                && !name.is_empty()
                && name.bytes().all(is_segment_byte)
                && is_secret_key(name)
            {
                return format!("{} {REDACTED}", &value[..=colon]);
            }
        }
    }
    let value = redact_bearer(&value);
    redact_raw_credentials(&value)
}

/// Return `tokens` with credential-like values replaced by [`REDACTED`].
///
/// Applies the same rules as `srb.utils.redaction.redact_secret_arguments`:
/// `KEY=VALUE` pairs whose key is credential vocabulary keep the key and lose
/// the value; a standalone credential flag (`--wandb-api-key`, but not a
/// `--no-*` switch) has the following token replaced unless that token is
/// exactly an option spelling (`--headless`, `-v`, `--`); every other value
/// passes through [`redact_secret_text`]. Intended only for log and preview
/// strings; the executed argv must never pass through this function.
#[must_use]
pub fn redact_secret_tokens(tokens: &[String]) -> Vec<String> {
    let mut redacted = Vec::with_capacity(tokens.len());
    let mut expect_secret_value = false;
    for token in tokens {
        if expect_secret_value {
            expect_secret_value = false;
            if !is_exact_option_token(token) {
                redacted.push(REDACTED.to_string());
                continue;
            }
        }
        match token.split_once('=') {
            Some((key, _value)) if is_key_shaped(key) && is_secret_key(key) => {
                redacted.push(format!("{key}={REDACTED}"));
            }
            Some((key, value)) if is_key_shaped(key) => {
                redacted.push(format!("{key}={}", redact_secret_text(value)));
            }
            _ => {
                if !token.contains('=')
                    && is_flag_token(token)
                    && is_secret_key(token)
                    && !is_negated_flag(token)
                {
                    expect_secret_value = true;
                }
                redacted.push(redact_secret_text(token));
            }
        }
    }
    redacted
}

/// The string is split back into its shell tokens (honouring the single-quote
/// form emitted by this module), redacted with [`redact_secret_tokens`], and
/// re-quoted. Only use the result for logging or display.
#[must_use]
pub fn redact_shell_command(command: &str) -> String {
    redact_secret_tokens(&split_shell_tokens(command))
        .iter()
        .map(|token| quote_shell_token(token))
        .collect::<Vec<_>>()
        .join(" ")
}

fn split_shell_tokens(command: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut current = String::new();
    let mut has_token = false;
    let mut in_quotes = false;
    let mut characters = command.chars().peekable();
    while let Some(character) = characters.next() {
        match character {
            '\'' => {
                in_quotes = !in_quotes;
                has_token = true;
            }
            '\\' if !in_quotes && characters.peek() == Some(&'\'') => {
                // `'\''` is the escaped single quote emitted by `quote_shell_token`.
                characters.next();
                current.push('\'');
                has_token = true;
            }
            ' ' if !in_quotes => {
                if has_token {
                    tokens.push(std::mem::take(&mut current));
                    has_token = false;
                }
            }
            _ => {
                current.push(character);
                has_token = true;
            }
        }
    }
    if has_token {
        tokens.push(current);
    }
    tokens
}

fn quote_shell_token(token: &str) -> String {
    if token.is_empty() {
        return "''".to_string();
    }

    if token.bytes().all(is_shell_safe_byte) {
        token.to_string()
    } else {
        format!("'{}'", token.replace('\'', "'\\''"))
    }
}

fn is_shell_safe_byte(byte: u8) -> bool {
    byte.is_ascii_alphanumeric()
        || matches!(
            byte,
            b'_' | b'-' | b'.' | b'/' | b':' | b'=' | b'+' | b',' | b'@' | b'%'
        )
}
