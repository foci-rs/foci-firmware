//! Parser for the `CommissionReplySink` manifest DSL consumed by
//! `commission_reply_trait_methods!` and `commission_reply_forwarding_impls!`.
//!
//! Each entry is:
//! ```text
//! entry {
//!     name: <method_ident>,
//!     doc: "<one or more doc lines, repeated `doc:` per line>",
//!     [param: <override parameter type path>,]
//!     reply: <wire reply type path, with lifetime if the type has one>,
//!     fields: { <field_ident>: <field_type>, ... },
//!     [no_default,]
//! }
//! ```
//! `param` is only present for the one entry (`foci_commission_timing`)
//! whose trait-method parameter type differs from its wire reply type; all
//! other entries use `reply` for both. `no_default` marks the one entry
//! (`foci_robustness_cycle_evidence`) that must stay a required trait
//! method (no default body) even though its forwarding body is generated
//! like every other entry.

use syn::{
    Ident, LitStr, Path, PathArguments, Token, Type, braced,
    parse::{Parse, ParseStream},
    punctuated::Punctuated,
};

mod kw {
    syn::custom_keyword!(entry);
    syn::custom_keyword!(name);
    syn::custom_keyword!(doc);
    syn::custom_keyword!(param);
    syn::custom_keyword!(reply);
    syn::custom_keyword!(fields);
    syn::custom_keyword!(no_default);
}

pub struct Field {
    pub name: Ident,
    pub ty: Type,
}

impl Parse for Field {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let name: Ident = input.parse()?;
        input.parse::<Token![:]>()?;
        let ty: Type = input.parse()?;
        Ok(Field { name, ty })
    }
}

pub struct Entry {
    pub name: Ident,
    pub doc_lines: Vec<LitStr>,
    pub param_override: Option<Type>,
    pub reply: Type,
    pub fields: Vec<Field>,
    pub no_default: bool,
}

impl Entry {
    /// The type used as the generated method's parameter: the `param`
    /// override when present, otherwise `reply` itself.
    pub fn param_type(&self) -> &Type {
        self.param_override.as_ref().unwrap_or(&self.reply)
    }

    /// `reply` with any generic arguments stripped from its final path
    /// segment. `klipper_reply_from!` substitutes this into struct-literal
    /// position (`#path { field: value, .. }`), where a bare `<'_>` parses
    /// as an attempted loop label rather than an elided lifetime; the
    /// pre-existing hand-written call sites always passed a bare path here
    /// for the same reason, relying on lifetime elision.
    pub fn reply_path(&self) -> Path {
        let Type::Path(type_path) = &self.reply else {
            panic!("reply type must be a path type");
        };
        let mut path = type_path.path.clone();
        if let Some(segment) = path.segments.last_mut() {
            segment.arguments = PathArguments::None;
        }
        path
    }
}

impl Parse for Entry {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        input.parse::<kw::entry>()?;
        let content;
        braced!(content in input);

        let mut name = None;
        let mut doc_lines = Vec::new();
        let mut param_override = None;
        let mut reply = None;
        let mut fields = Vec::new();
        let mut no_default = false;

        while !content.is_empty() {
            if content.peek(kw::name) {
                content.parse::<kw::name>()?;
                content.parse::<Token![:]>()?;
                name = Some(content.parse::<Ident>()?);
            } else if content.peek(kw::doc) {
                content.parse::<kw::doc>()?;
                content.parse::<Token![:]>()?;
                doc_lines.push(content.parse::<LitStr>()?);
            } else if content.peek(kw::param) {
                content.parse::<kw::param>()?;
                content.parse::<Token![:]>()?;
                param_override = Some(content.parse::<Type>()?);
            } else if content.peek(kw::reply) {
                content.parse::<kw::reply>()?;
                content.parse::<Token![:]>()?;
                reply = Some(content.parse::<Type>()?);
            } else if content.peek(kw::fields) {
                content.parse::<kw::fields>()?;
                content.parse::<Token![:]>()?;
                let fields_content;
                braced!(fields_content in content);
                let parsed: Punctuated<Field, Token![,]> =
                    fields_content.parse_terminated(Field::parse, Token![,])?;
                fields = parsed.into_iter().collect();
            } else if content.peek(kw::no_default) {
                content.parse::<kw::no_default>()?;
                no_default = true;
            } else {
                return Err(content.error("unknown manifest entry field"));
            }
            let _ = content.parse::<Token![,]>();
        }

        Ok(Entry {
            name: name.ok_or_else(|| input.error("entry missing `name`"))?,
            doc_lines,
            param_override,
            reply: reply.ok_or_else(|| input.error("entry missing `reply`"))?,
            fields,
            no_default,
        })
    }
}

pub struct Manifest {
    pub entries: Vec<Entry>,
}

impl Parse for Manifest {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut entries = Vec::new();
        while !input.is_empty() {
            entries.push(input.parse()?);
        }
        Ok(Manifest { entries })
    }
}
