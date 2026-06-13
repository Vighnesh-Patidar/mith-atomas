#include "doctest.h"

#include "mith/core/json_writer.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

using mith::JsonWriter;

TEST_CASE("default-constructed JsonWriter is empty and well-formed") {
    JsonWriter w;
    CHECK(w.str().empty());
    CHECK(w.well_formed());
}

TEST_CASE("top-level scalar values") {
    SUBCASE("null") {
        JsonWriter w; w.write_null();
        CHECK(w.str() == "null");
        CHECK(w.well_formed());
    }
    SUBCASE("true / false") {
        JsonWriter w; w.write_bool(true);
        CHECK(w.str() == "true");
        w.clear();
        w.write_bool(false);
        CHECK(w.str() == "false");
    }
    SUBCASE("signed int") {
        JsonWriter w; w.write_i64(-42);
        CHECK(w.str() == "-42");
    }
    SUBCASE("unsigned int — max uint64") {
        JsonWriter w; w.write_u64(std::numeric_limits<std::uint64_t>::max());
        CHECK(w.str() == "18446744073709551615");
    }
    SUBCASE("plain string") {
        JsonWriter w; w.write_string("hello");
        CHECK(w.str() == "\"hello\"");
    }
}

TEST_CASE("empty containers") {
    SUBCASE("empty object") {
        JsonWriter w; w.begin_object(); w.end_object();
        CHECK(w.str() == "{}");
        CHECK(w.well_formed());
    }
    SUBCASE("empty array") {
        JsonWriter w; w.begin_array(); w.end_array();
        CHECK(w.str() == "[]");
        CHECK(w.well_formed());
    }
}

TEST_CASE("object: one key-value") {
    JsonWriter w;
    w.begin_object();
    w.key("a"); w.write_i64(1);
    w.end_object();
    CHECK(w.str() == "{\"a\":1}");
    CHECK(w.well_formed());
}

TEST_CASE("object: comma between multiple key-values") {
    JsonWriter w;
    w.begin_object();
    w.key("a"); w.write_i64(1);
    w.key("b"); w.write_string("x");
    w.key("c"); w.write_bool(true);
    w.end_object();
    CHECK(w.str() == "{\"a\":1,\"b\":\"x\",\"c\":true}");
}

TEST_CASE("array: comma between elements") {
    JsonWriter w;
    w.begin_array();
    w.write_i64(1);
    w.write_i64(2);
    w.write_i64(3);
    w.end_array();
    CHECK(w.str() == "[1,2,3]");
}

TEST_CASE("nested: array inside object") {
    JsonWriter w;
    w.begin_object();
    w.key("xs"); w.begin_array();
        w.write_i64(1);
        w.write_i64(2);
    w.end_array();
    w.end_object();
    CHECK(w.str() == "{\"xs\":[1,2]}");
}

TEST_CASE("nested: object inside object") {
    JsonWriter w;
    w.begin_object();
    w.key("inner"); w.begin_object();
        w.key("v"); w.write_i64(7);
    w.end_object();
    w.end_object();
    CHECK(w.str() == "{\"inner\":{\"v\":7}}");
}

TEST_CASE("nested: object inside array, with commas at the right places") {
    JsonWriter w;
    w.begin_array();
        w.begin_object();
            w.key("a"); w.write_i64(1);
        w.end_object();
        w.begin_object();
            w.key("b"); w.write_i64(2);
        w.end_object();
    w.end_array();
    CHECK(w.str() == "[{\"a\":1},{\"b\":2}]");
}

TEST_CASE("nested: multi-level with mixed scopes and trailing scalars") {
    JsonWriter w;
    w.begin_object();
    w.key("level");  w.write_string("info");
    w.key("payload"); w.begin_object();
        w.key("tags"); w.begin_array();
            w.write_string("a");
            w.write_string("b");
        w.end_array();
        w.key("count"); w.write_u64(42);
    w.end_object();
    w.key("ok"); w.write_bool(true);
    w.end_object();

    CHECK(w.str() ==
        R"({"level":"info","payload":{"tags":["a","b"],"count":42},"ok":true})");
}

TEST_CASE("string escaping: quote, backslash, control chars") {
    JsonWriter w;
    w.write_string("he said \"hi\"");
    CHECK(w.str() == "\"he said \\\"hi\\\"\"");

    w.clear();
    w.write_string("a\\b");
    CHECK(w.str() == "\"a\\\\b\"");

    w.clear();
    w.write_string("line1\nline2\ttab");
    CHECK(w.str() == "\"line1\\nline2\\ttab\"");

    w.clear();
    std::string with_ctrl = "x";
    with_ctrl.push_back(static_cast<char>(0x01));   // SOH
    with_ctrl.push_back('y');
    w.write_string(with_ctrl);
    CHECK(w.str() == "\"x\\u0001y\"");
}

TEST_CASE("f64: finite formatting and NaN / Infinity → null") {
    JsonWriter w;

    w.write_f64(3.14);
    // Don't hard-code the exact representation (impl-defined %.17g),
    // just check it parses to something close.
    const std::string& s = w.str();
    CHECK(s.find("3.14") != std::string::npos);

    w.clear();
    w.write_f64(std::numeric_limits<double>::quiet_NaN());
    CHECK(w.str() == "null");

    w.clear();
    w.write_f64(std::numeric_limits<double>::infinity());
    CHECK(w.str() == "null");

    w.clear();
    w.write_f64(-std::numeric_limits<double>::infinity());
    CHECK(w.str() == "null");
}

TEST_CASE("newline() appends a literal \\n to the buffer") {
    JsonWriter w;
    w.begin_object(); w.end_object();
    w.newline();
    CHECK(w.str() == "{}\n");
}

TEST_CASE("take() yields the buffer and resets the writer") {
    JsonWriter w;
    w.begin_object();
    w.key("a"); w.write_i64(1);
    w.end_object();

    const std::string out = w.take();
    CHECK(out == "{\"a\":1}");
    CHECK(w.str().empty());
    CHECK(w.well_formed());
}

TEST_CASE("clear() resets state, allowing reuse") {
    JsonWriter w;
    w.begin_object();
    w.key("a"); w.write_i64(1);
    w.end_object();
    REQUIRE(w.str() == "{\"a\":1}");

    w.clear();
    CHECK(w.str().empty());
    CHECK(w.well_formed());

    w.begin_array(); w.write_i64(99); w.end_array();
    CHECK(w.str() == "[99]");
}

TEST_CASE("well_formed: true only when all scopes are closed") {
    JsonWriter w;
    CHECK(w.well_formed());

    w.begin_object();
    CHECK_FALSE(w.well_formed());

    w.end_object();
    CHECK(w.well_formed());

    w.begin_array();
    w.begin_object();
    CHECK_FALSE(w.well_formed());
    w.end_object();
    CHECK_FALSE(w.well_formed());
    w.end_array();
    CHECK(w.well_formed());
}

TEST_CASE("negative i64 and zero") {
    JsonWriter w;
    w.begin_array();
    w.write_i64(0);
    w.write_i64(std::numeric_limits<std::int64_t>::min());
    w.write_i64(std::numeric_limits<std::int64_t>::max());
    w.end_array();

    const std::string expected =
        std::string("[0,") + std::to_string(std::numeric_limits<std::int64_t>::min())
                  + "," + std::to_string(std::numeric_limits<std::int64_t>::max())
                  + "]";
    CHECK(w.str() == expected);
}
