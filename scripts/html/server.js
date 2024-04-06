const express = require("express");
const { Pool } = require("pg");
const bodyParser = require("body-parser");

const app = express();
app.use(bodyParser.urlencoded({ extended: true }));

const pool = new Pool({
  user: "postgres",
  host: "localhost",
  database: "survey_responses",
  password: "postgres",
  port: 5432,
});

app.use(express.static("public"));

app.get("/", (req, res) => {
  res.sendFile(__dirname + "/survey1.html");
});

app.get("/survey1", (req, res) => {
  res.sendFile(__dirname + "/survey1.html");
});

app.post("/submit_survey1", (req, res) => {
  const formData = req.body;

  const createTableQuery = `
    CREATE TABLE IF NOT EXISTS survey1.responses (
      id SERIAL PRIMARY KEY,
      consent boolean,
      countryOfBirth TEXT,
      longTimeCountry TEXT,
      currentCountryOfResidence TEXT,
      culturalBackgroundIdentification TEXT
    );
  `;

  pool.query(createTableQuery, (error, results) => {
    if (error) {
      console.error("Error creating table:", error);
      return;
    }

    // console.log("Table created successfully");
  });

  pool.query(
    "INSERT INTO survey1.responses (consent, countryOfBirth, longTimeCountry, currentCountryOfResidence, culturalBackgroundIdentification) VALUES ($1, $2, $3, $4, $5)",
    [
      formData.consent,
      formData.countryOfBirth,
      formData.longTimeCountry,
      formData.currentCountryOfResidence,
      formData.culturalBackgroundIdentification,
    ],
    (error, results) => {
      if (error) {
        console.log("Error inserting into database:", error);
        res.status(500).json({ error: "Database error" });
        return;
      }
      res.redirect("/thankYou.html");
    }
  );
});

app.get("/survey2", (req, res) => {
  res.sendFile(__dirname + "/survey2.html");
});

app.post("/submit_survey2", (req, res) => {
  const formData = req.body;

  pool.query(
    "INSERT INTO survey2.responses (response) VALUES ($1)",
    [formData],
    (error, results) => {
      if (error) {
        res.status(500).json({ error: "Database error" });
        return;
      }
      res.redirect("/thankYou.html");
    }
  );
});

app.get("/thankYou.html", (req, res) => {
  res.sendFile(__dirname + "/thankYou.html");
});

app.listen(3000, () => {
  console.log("Server is running on port 3000");
});
